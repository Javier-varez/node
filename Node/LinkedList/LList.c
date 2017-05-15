
#include "LList.h"
#include <stdlib.h>

LListElement *LList_CreateList(void *content) {
  LListElement * head = malloc(sizeof(LListElement));
  if (head) {
    head->content = content;
    head->nextElement = NULL;
  }
  return head;
}

void LList_AppendElement(LListElement *head, void *content) {
  if (head == NULL) return; // Return if head doesn't exist.

  LListElement *newElement = malloc(sizeof(LListElement));

  // If new element was correctly allocated.
  if (newElement) {
    // Set next element to NULL, and point to content
    newElement->nextElement = NULL;
    newElement->content = content;

    // find last element of the list
    while (head->nextElement) {
      head = head->nextElement;
    }

    // Set the new element as the last of the list.
    head->nextElement = newElement;
  }
}

void* LList_GetElement(LListElement *head, uint32_t position) {
  if (head == NULL) return NULL; // Return NULL if head doesn't exist.

  uint32_t index;
  for (index = 0; index < position; index++) {
    // If the index surpasses the boundaries of the list, return NULL
    if (head->nextElement == NULL) return NULL;
    head = head->nextElement;
  }

  return head->content;
}

uint32_t LList_CountElements(LListElement *head) {
  uint32_t count = 0;
  while(head) {
    head = head->nextElement;
    count++;
  }
  return count;
}

// Careful, contents should be destroyed prior to destroying the list.
void LList_DestroyList(LListElement *head) {
  while (head) {
    LListElement *tmp = head->nextElement;
    free(head);
    head = tmp;
  }
}
