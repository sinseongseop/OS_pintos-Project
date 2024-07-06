#ifndef _PROJECTS_PROJECT1_AW_THREAD_H__
#define _PROJECTS_PROJECT1_AW_THREAD_H__

#include "threads/interrupt.h"
#include "threads/synch.h"
#include "threads/thread.h"

extern struct list blocked_threads;

void block_thread();

void unblock_threads();

#endif