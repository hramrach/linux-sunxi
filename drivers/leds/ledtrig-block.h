#ifndef __LEDTRIG_BLOCK_H_INCLUDED
#define __LEDTRIG_BLOCK_H_INCLUDED
struct gendisk;
void ledtrig_block_add(struct gendisk *);
void ledtrig_block_del(struct gendisk *);
#endif
