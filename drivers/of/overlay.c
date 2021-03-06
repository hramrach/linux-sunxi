/*
 * Functions for working with device tree overlays
 *
 * Copyright (C) 2012 Pantelis Antoniou <panto@antoniou-consulting.com>
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#undef DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/sysfs.h>
#include <linux/atomic.h>

#include "of_private.h"

/**
 * struct of_overlay_info - Holds a single overlay info
 * @target:	target of the overlay operation
 * @overlay:	pointer to the overlay contents node
 *
 * Holds a single overlay state, including all the overlay logs &
 * records.
 */
struct of_overlay_info {
	struct device_node *target;
	struct device_node *overlay;
};

/**
 * struct of_overlay - Holds a complete overlay transaction
 * @node:	List on which we are located
 * @count:	Count of ovinfo structures
 * @ovinfo_tab:	Overlay info table (count sized)
 * @cset:	Changeset to be used
 *
 * Holds a complete overlay transaction
 */
struct of_overlay {
	int id;
	struct list_head node;
	int count;
	struct of_overlay_info *ovinfo_tab;
	struct of_changeset cset;
	struct kobject kobj;
};


static struct kobject *ov_control_kobj;
/* master enable switch; once set to 0 can't be re-enabled */
static atomic_t ov_enable = ATOMIC_INIT(1);

static int of_overlay_apply_one(struct of_overlay *ov,
		struct device_node *target, const struct device_node *overlay);
static int overlay_removal_is_ok(struct of_overlay *ov);

static int of_overlay_apply_single_property(struct of_overlay *ov,
		struct device_node *target, struct property *prop)
{
	struct property *propn, *tprop;

	/* NOTE: Multiple changes of single properties not supported */
	tprop = of_find_property(target, prop->name, NULL);

	/* special properties are not meant to be updated (silent NOP) */
	if (of_prop_cmp(prop->name, "name") == 0 ||
	    of_prop_cmp(prop->name, "phandle") == 0 ||
	    of_prop_cmp(prop->name, "linux,phandle") == 0)
		return 0;

	propn = __of_prop_dup(prop, GFP_KERNEL);
	if (propn == NULL)
		return -ENOMEM;

	/* not found? add */
	if (tprop == NULL)
		return of_changeset_add_property(&ov->cset, target, propn);

	/* found? update */
	return of_changeset_update_property(&ov->cset, target, propn);
}

static int of_overlay_apply_single_device_node(struct of_overlay *ov,
		struct device_node *target, struct device_node *child)
{
	const char *cname;
	struct device_node *tchild;
	int ret = 0;

	cname = kbasename(child->full_name);
	if (cname == NULL)
		return -ENOMEM;

	/* NOTE: Multiple mods of created nodes not supported */
	tchild = of_get_child_by_name(target, cname);
	if (tchild != NULL) {
		/* apply overlay recursively */
		ret = of_overlay_apply_one(ov, tchild, child);
		of_node_put(tchild);
	} else {
		/* create empty tree as a target */
		tchild = __of_node_dup(child, "%s/%s", target->full_name, cname);
		if (!tchild)
			return -ENOMEM;

		/* point to parent */
		tchild->parent = target;

		ret = of_changeset_attach_node(&ov->cset, tchild);
		if (ret)
			return ret;

		ret = of_overlay_apply_one(ov, tchild, child);
		if (ret)
			return ret;
	}

	return ret;
}

/*
 * Apply a single overlay node recursively.
 *
 * Note that the in case of an error the target node is left
 * in a inconsistent state. Error recovery should be performed
 * by using the changeset.
 */
static int of_overlay_apply_one(struct of_overlay *ov,
		struct device_node *target, const struct device_node *overlay)
{
	struct device_node *child;
	struct property *prop;
	int ret;

	for_each_property_of_node(overlay, prop) {
		ret = of_overlay_apply_single_property(ov, target, prop);
		if (ret) {
			pr_err("%s: Failed to apply prop @%s/%s\n",
				__func__, target->full_name, prop->name);
			return ret;
		}
	}

	for_each_child_of_node(overlay, child) {
		ret = of_overlay_apply_single_device_node(ov, target, child);
		if (ret != 0) {
			pr_err("%s: Failed to apply single node @%s/%s\n",
					__func__, target->full_name,
					child->name);
			return ret;
		}
	}

	return 0;
}

/**
 * of_overlay_apply() - Apply @count overlays pointed at by @ovinfo_tab
 * @ov:		Overlay to apply
 *
 * Applies the overlays given, while handling all error conditions
 * appropriately. Either the operation succeeds, or if it fails the
 * live tree is reverted to the state before the attempt.
 * Returns 0, or an error if the overlay attempt failed.
 */
static int of_overlay_apply(struct of_overlay *ov)
{
	int i, err;

	/* first we apply the overlays atomically */
	for (i = 0; i < ov->count; i++) {
		struct of_overlay_info *ovinfo = &ov->ovinfo_tab[i];

		err = of_overlay_apply_one(ov, ovinfo->target, ovinfo->overlay);
		if (err != 0) {
			pr_err("%s: overlay failed '%s'\n",
				__func__, ovinfo->target->full_name);
			return err;
		}
	}

	return 0;
}

/*
 * Find the target node using a number of different strategies
 * in order of preference
 *
 * "target" property containing the phandle of the target
 * "target-path" property containing the path of the target
 */
static struct device_node *find_target_node(struct device_node *info_node)
{
	const char *path;
	u32 val;
	int ret;

	/* first try to go by using the target as a phandle */
	ret = of_property_read_u32(info_node, "target", &val);
	if (ret == 0)
		return of_find_node_by_phandle(val);

	/* now try to locate by path */
	ret = of_property_read_string(info_node, "target-path", &path);
	if (ret == 0)
		return of_find_node_by_path(path);

	pr_err("%s: Failed to find target for node %p (%s)\n", __func__,
		info_node, info_node->name);

	return NULL;
}

/**
 * of_fill_overlay_info() - Fill an overlay info structure
 * @ov		Overlay to fill
 * @info_node:	Device node containing the overlay
 * @ovinfo:	Pointer to the overlay info structure to fill
 *
 * Fills an overlay info structure with the overlay information
 * from a device node. This device node must have a target property
 * which contains a phandle of the overlay target node, and an
 * __overlay__ child node which has the overlay contents.
 * Both ovinfo->target & ovinfo->overlay have their references taken.
 *
 * Returns 0 on success, or a negative error value.
 */
static int of_fill_overlay_info(struct of_overlay *ov,
		struct device_node *info_node, struct of_overlay_info *ovinfo)
{
	pr_debug("%s: info for %pOfp (%s)", __func__,
	       info_node, info_node->name);

	ovinfo->overlay = of_get_child_by_name(info_node, "__overlay__");
	if (ovinfo->overlay == NULL)
		goto err_fail;
	pr_debug(" overlay %pOfp (%s)",
	       ovinfo->overlay, ovinfo->overlay->name);

	ovinfo->target = find_target_node(info_node);
	if (ovinfo->target == NULL)
		goto err_fail;
	pr_debug(" target %pOfp (%s)",
	       ovinfo->target, ovinfo->target->name);

	return 0;

err_fail:
	of_node_put(ovinfo->target);
	of_node_put(ovinfo->overlay);

	memset(ovinfo, 0, sizeof(*ovinfo));
	return -EINVAL;
}

/**
 * of_build_overlay_info() - Build an overlay info array
 * @ov		Overlay to build
 * @tree:	Device node containing all the overlays
 *
 * Helper function that given a tree containing overlay information,
 * allocates and builds an overlay info array containing it, ready
 * for use using of_overlay_apply.
 *
 * Returns 0 on success with the @cntp @ovinfop pointers valid,
 * while on error a negative error value is returned.
 */
static int of_build_overlay_info(struct of_overlay *ov,
		struct device_node *tree)
{
	struct device_node *node;
	struct of_overlay_info *ovinfo;
	int cnt, err;

	/* worst case; every child is a node */
	cnt = 0;
	for_each_child_of_node(tree, node)
		cnt++;

	ovinfo = kcalloc(cnt, sizeof(*ovinfo), GFP_KERNEL);
	if (ovinfo == NULL)
		return -ENOMEM;

	cnt = 0;
	for_each_child_of_node(tree, node) {
		memset(&ovinfo[cnt], 0, sizeof(*ovinfo));
		err = of_fill_overlay_info(ov, node, &ovinfo[cnt]);
		if (err == 0)
			cnt++;
	}

	/* if nothing filled, return error */
	if (cnt == 0) {
		kfree(ovinfo);
		return -ENODEV;
	}

	ov->count = cnt;
	ov->ovinfo_tab = ovinfo;

	return 0;
}

/**
 * of_free_overlay_info() - Free an overlay info array
 * @ov		Overlay to free the overlay info from
 * @ovinfo_tab:	Array of overlay_info's to free
 *
 * Releases the memory of a previously allocated ovinfo array
 * by of_build_overlay_info.
 * Returns 0, or an error if the arguments are bogus.
 */
static int of_free_overlay_info(struct of_overlay *ov)
{
	struct of_overlay_info *ovinfo;
	int i;

	/* do it in reverse */
	for (i = ov->count - 1; i >= 0; i--) {
		ovinfo = &ov->ovinfo_tab[i];

		of_node_put(ovinfo->target);
		of_node_put(ovinfo->overlay);
	}
	kfree(ov->ovinfo_tab);

	return 0;
}

static LIST_HEAD(ov_list);
static DEFINE_IDR(ov_idr);

static inline struct of_overlay *kobj_to_overlay(struct kobject *kobj)
{
	return container_of(kobj, struct of_overlay, kobj);
}

void of_overlay_release(struct kobject *kobj)
{
	struct of_overlay *ov = kobj_to_overlay(kobj);

	kfree(ov);
}

static ssize_t enable_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&ov_enable));
}

static ssize_t enable_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	long new_enable;

	ret = kstrtol(buf, 10, &new_enable);
	if (ret != 0)
		return ret;
	if ((unsigned long)new_enable > 1)
		return -EINVAL;
	/* if we've disabled it, no going back */
	if (atomic_read(&ov_enable) == 0)
		return -EPERM;
	atomic_set(&ov_enable, (int)new_enable);
	if(!new_enable)
		kobject_put(ov_control_kobj);
	return count;
}

static struct kobj_attribute enable_attr = __ATTR_RW(enable);

static const struct attribute *overlay_global_attrs[] = {
	&enable_attr.attr,
	NULL
};

static ssize_t can_remove_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct of_overlay *ov = kobj_to_overlay(kobj);

	return snprintf(buf, PAGE_SIZE, "%d\n", overlay_removal_is_ok(ov));
}

static ssize_t targets_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct of_overlay *ov = kobj_to_overlay(kobj);
	struct of_overlay_info *ovinfo;
	char *s, *e;
	ssize_t ret;
	int i, len;

	s = buf;
	e = buf + PAGE_SIZE;

	mutex_lock(&of_mutex);

	/* targets */
	for (i = 0; i < ov->count; i++) {
		ovinfo = &ov->ovinfo_tab[i];

		len = snprintf(s, e - s, "%s\n",
				of_node_full_name(ovinfo->target));
		if (len == 0) {
			ret = -ENOSPC;
			goto err;
		}
		s += len;
	}

	/* the buffer is zero terminated */
	ret = s - buf;
err:
	mutex_unlock(&of_mutex);
	return ret;
}

static struct kobj_attribute can_remove_attr = __ATTR_RO(can_remove);
static struct kobj_attribute targets_attr = __ATTR_RO(targets);

static struct attribute *overlay_attrs[] = {
	&can_remove_attr.attr,
	&targets_attr.attr,
	NULL
};

static struct kobj_type of_overlay_ktype = {
	.release = of_overlay_release,
	.sysfs_ops = &kobj_sysfs_ops,	/* default kobj sysfs ops */
	.default_attrs = overlay_attrs,
};

static struct kset *ov_kset;

/**
 * of_overlay_create() - Create and apply an overlay
 * @tree:	Device node containing all the overlays
 *
 * Creates and applies an overlay while also keeping track
 * of the overlay in a list. This list can be used to prevent
 * illegal overlay removals.
 *
 * Returns the id of the created overlay, or a negative error number
 */
int of_overlay_create(struct device_node *tree)
{
	struct of_overlay *ov;
	int err, id;

	/* administratively disabled */
	if (!atomic_read(&ov_enable))
		return -EPERM;

	/* allocate the overlay structure */
	ov = kzalloc(sizeof(*ov), GFP_KERNEL);
	if (ov == NULL)
		return -ENOMEM;
	ov->id = -1;

	INIT_LIST_HEAD(&ov->node);

	of_changeset_init(&ov->cset);

	/* initialize kobject */
	kobject_init(&ov->kobj, &of_overlay_ktype);

	mutex_lock(&of_mutex);

	id = idr_alloc(&ov_idr, ov, 0, 0, GFP_KERNEL);
	if (id < 0) {
		pr_err("%s: idr_alloc() failed for tree@%s\n",
				__func__, tree->full_name);
		err = id;
		goto err_destroy_trans;
	}
	ov->id = id;

	/* build the overlay info structures */
	err = of_build_overlay_info(ov, tree);
	if (err) {
		pr_err("%s: of_build_overlay_info() failed for tree@%s\n",
				__func__, tree->full_name);
		goto err_free_idr;
	}

	/* apply the overlay */
	err = of_overlay_apply(ov);
	if (err) {
		pr_err("%s: of_overlay_apply() failed for tree@%s\n",
				__func__, tree->full_name);
		goto err_abort_trans;
	}

	/* apply the changeset */
	err = of_changeset_apply(&ov->cset);
	if (err) {
		pr_err("%s: of_changeset_apply() failed for tree@%s\n",
				__func__, tree->full_name);
		goto err_revert_overlay;
	}

	ov->kobj.kset = ov_kset;
	err = kobject_add(&ov->kobj, NULL, "%d", id);
	if (err != 0) {
		pr_err("%s: kobject_add() failed for tree@%s\n",
				__func__, tree->full_name);
		goto err_cancel_overlay;
	}

	/* add to the tail of the overlay list */
	list_add_tail(&ov->node, &ov_list);

	mutex_unlock(&of_mutex);

	return id;

err_cancel_overlay:
	of_changeset_revert(&ov->cset);
err_revert_overlay:
err_abort_trans:
	of_free_overlay_info(ov);
err_free_idr:
	idr_remove(&ov_idr, ov->id);
err_destroy_trans:
	of_changeset_destroy(&ov->cset);
	kfree(ov);
	mutex_unlock(&of_mutex);

	return err;
}
EXPORT_SYMBOL_GPL(of_overlay_create);

/* check whether the given node, lies under the given tree */
static int overlay_subtree_check(struct device_node *tree,
		struct device_node *dn)
{
	struct device_node *child;

	/* match? */
	if (tree == dn)
		return 1;

	for_each_child_of_node(tree, child) {
		if (overlay_subtree_check(child, dn))
			return 1;
	}

	return 0;
}

/* check whether this overlay is the topmost */
static int overlay_is_topmost(struct of_overlay *ov, struct device_node *dn)
{
	struct of_overlay *ovt;
	struct of_changeset_entry *ce;

	list_for_each_entry_reverse(ovt, &ov_list, node) {
		/* if we hit ourselves, we're done */
		if (ovt == ov)
			break;

		/* check against each subtree affected by this overlay */
		list_for_each_entry(ce, &ovt->cset.entries, node) {
			if (overlay_subtree_check(ce->np, dn)) {
				pr_err("%s: #%d clashes #%d @%s\n",
					__func__, ov->id, ovt->id,
					dn->full_name);
				return 0;
			}
		}
	}

	/* overlay is topmost */
	return 1;
}

/*
 * We can safely remove the overlay only if it's the top-most one.
 * Newly applied overlays are inserted at the tail of the overlay list,
 * so a top most overlay is the one that is closest to the tail.
 *
 * The topmost check is done by exploiting this property. For each
 * affected device node in the log list we check if this overlay is
 * the one closest to the tail. If another overlay has affected this
 * device node and is closest to the tail, then removal is not permited.
 */
static int overlay_removal_is_ok(struct of_overlay *ov)
{
	struct of_changeset_entry *ce;

	list_for_each_entry(ce, &ov->cset.entries, node) {
		if (!overlay_is_topmost(ov, ce->np)) {
			pr_err("%s: overlay #%d is not topmost\n",
					__func__, ov->id);
			return 0;
		}
	}

	return 1;
}

/**
 * of_overlay_destroy() - Removes an overlay
 * @id:	Overlay id number returned by a previous call to of_overlay_create
 *
 * Removes an overlay if it is permissible.
 *
 * Returns 0 on success, or a negative error number
 */
int of_overlay_destroy(int id)
{
	struct of_overlay *ov;
	int err;

	mutex_lock(&of_mutex);

	ov = idr_find(&ov_idr, id);
	if (ov == NULL) {
		err = -ENODEV;
		pr_err("%s: Could not find overlay #%d\n",
				__func__, id);
		goto out;
	}

	/* check whether the overlay is safe to remove */
	if (!overlay_removal_is_ok(ov)) {
		err = -EBUSY;
		pr_err("%s: removal check failed for overlay #%d\n",
				__func__, id);
		goto out;
	}

	list_del(&ov->node);
	of_changeset_revert(&ov->cset);
	of_free_overlay_info(ov);
	idr_remove(&ov_idr, id);
	of_changeset_destroy(&ov->cset);

	kobject_del(&ov->kobj);
	kobject_put(&ov->kobj);

	err = 0;

out:
	mutex_unlock(&of_mutex);

	return err;
}
EXPORT_SYMBOL_GPL(of_overlay_destroy);

/**
 * of_overlay_destroy_all() - Removes all overlays from the system
 *
 * Removes all overlays from the system in the correct order.
 *
 * Returns 0 on success, or a negative error number
 */
int of_overlay_destroy_all(void)
{
	struct of_overlay *ov, *ovn;

	mutex_lock(&of_mutex);

	/* the tail of list is guaranteed to be safe to remove */
	list_for_each_entry_safe_reverse(ov, ovn, &ov_list, node) {
		list_del(&ov->node);
		of_changeset_revert(&ov->cset);
		of_free_overlay_info(ov);
		idr_remove(&ov_idr, ov->id);
		kobject_del(&ov->kobj);
		kobject_put(&ov->kobj);
	}

	mutex_unlock(&of_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(of_overlay_destroy_all);

static ssize_t of_overlay_list_show(struct kobject *kobj, struct kobj_attribute
				    *attr, char *buf)
{
	struct of_overlay *ov, *ovn;
	ssize_t size = 0;
	int j;

	mutex_lock(&of_mutex);

	list_for_each_entry_safe_reverse(ov, ovn, &ov_list, node) {
		size += scnprintf(buf + size, PAGE_SIZE - size, "%d", ov->id);
		for (j = 0; j < ov->count; j++)
			size += scnprintf(buf + size, PAGE_SIZE - size,
					  "  %pOfp (%s)",
					  ov->ovinfo_tab[j].target,
					  ov->ovinfo_tab[j].target->name);
		size += scnprintf(buf + size, PAGE_SIZE - size, "\n");
	}

	mutex_unlock(&of_mutex);
	return size;
}

static ssize_t of_overlay_destroy_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf,
					size_t count)
{
	int ret;
	int id;

	ret = kstrtoint(buf, 10, &id);
	if (ret < 0)
		return ret;

	ret = of_overlay_destroy(id);
	if (ret < 0)
		return ret;
	return count;
}

static ssize_t of_overlay_destroy_all_store(struct kobject *kobj,
					    struct kobj_attribute *attr,
					    const char *buf,
					    size_t count)
{
	int ret = of_overlay_destroy_all();

	if (ret < 0)
		return ret;
	return count;
}

static ssize_t of_overlay_create_store(struct kobject *kobj,
				       struct kobj_attribute *attr,
				       const char *_buf,
				       size_t count)
{
	int ret;
	const unsigned long *buf = (const unsigned long *)_buf;
	struct device_node *tree = NULL;

	of_fdt_unflatten_tree(buf, &tree);
	if (!tree)
		return -EINVAL;
	of_resolve_phandles(tree);
	ret = of_overlay_create(tree);
	if (ret < 0)
		return ret;
	return count;
}

static struct kobj_attribute of_overlay_list_attr =
__ATTR(list, 0444, of_overlay_list_show, NULL);
static struct kobj_attribute of_overlay_create_attr =
__ATTR(create, 0200, NULL, of_overlay_create_store);
static struct kobj_attribute of_overlay_destroy_attr =
__ATTR(destroy, 0200, NULL, of_overlay_destroy_store);
static struct kobj_attribute of_overlay_destroy_all_attr =
__ATTR(destroy_all, 0200, NULL, of_overlay_destroy_all_store);

static struct attribute *of_overlay_attrs[] = {
		&of_overlay_list_attr.attr,
		&of_overlay_create_attr.attr,
		&of_overlay_destroy_attr.attr,
		&of_overlay_destroy_all_attr.attr,
		NULL
};

struct attribute_group of_overlay_attr_group = {
		.attrs = of_overlay_attrs,
};

/* called from of_init() */
int of_overlay_init(void)
{
	int retval;

	ov_kset = kset_create_and_add("overlays", NULL, &of_kset->kobj);
	if (!ov_kset) {
		return -ENOMEM;
	}

	ov_control_kobj = kobject_create_and_add("control", &ov_kset->kobj);
	if (!ov_control_kobj)
		return -ENOMEM;

	retval = sysfs_create_group(ov_control_kobj, &of_overlay_attr_group);
	WARN(retval, "%s: error adding control interface\n", __func__);

	retval = sysfs_create_files(&ov_kset->kobj, overlay_global_attrs);
	WARN(retval, "%s: error adding global attributes\n", __func__);

	return 0;
}
