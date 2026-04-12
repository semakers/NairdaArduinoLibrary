#ifndef nairda_table_h
#define nairda_table_h

#include <stdint.h>
#include "extern_libraries/linked_list/linked_list.h"

typedef LinkedList<int32_t> nairda_row_t;
typedef LinkedList<nairda_row_t *> nairda_table_t;

void tableCreate(nairda_table_t *table, int rows, int cols, int32_t defaultVal);
void tableSet(nairda_table_t *table, int row, int col, int32_t value);
int32_t tableGet(nairda_table_t *table, int row, int col);
int tableHeight(nairda_table_t *table);
int tableWidth(nairda_table_t *table);
void tableAddRow(nairda_table_t *table);
void tableAddColumn(nairda_table_t *table);
void tableRemoveRow(nairda_table_t *table);
void tableRemoveColumn(nairda_table_t *table);

#endif
