

#ifndef LLIST_H
#define LLIST_H

#include <stdint.h>

typedef struct llist {
  void *content;
  struct llist *nextElement;
} LListElement;


LListElement *LList_CreateList(void *content);
void LList_AppendElement(LListElement *head, void *content);
void* LList_GetElement(LListElement *head, uint32_t position);
uint32_t LList_CountElements(LListElement *head);
void LList_DestroyList(LListElement *head);

#endif
