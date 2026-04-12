#include "nairda_table.h"

void tableCreate(nairda_table_t *table, int rows, int cols, int32_t defaultVal) {
  for (int r = 0; r < rows; r++) {
    nairda_row_t *row = new nairda_row_t();
    for (int c = 0; c < cols; c++) {
      row->add(defaultVal);
    }
    table->add(row);
  }
}

void tableSet(nairda_table_t *table, int row, int col, int32_t value) {
  if (row < 0 || row >= table->size()) return;
  nairda_row_t *r = table->get(row);
  if (r == NULL || col < 0 || col >= r->size()) return;
  r->set(col, value);
}

int32_t tableGet(nairda_table_t *table, int row, int col) {
  if (row < 0 || row >= table->size()) return 0;
  nairda_row_t *r = table->get(row);
  if (r == NULL || col < 0 || col >= r->size()) return 0;
  return r->get(col);
}

int tableHeight(nairda_table_t *table) {
  return table->size();
}

int tableWidth(nairda_table_t *table) {
  if (table->size() == 0) return 0;
  nairda_row_t *firstRow = table->get(0);
  if (firstRow == NULL) return 0;
  return firstRow->size();
}

void tableAddRow(nairda_table_t *table) {
  int cols = tableWidth(table);
  nairda_row_t *row = new nairda_row_t();
  for (int c = 0; c < cols; c++) {
    row->add((int32_t)0);
  }
  table->add(row);
}

void tableAddColumn(nairda_table_t *table) {
  for (int r = 0; r < table->size(); r++) {
    nairda_row_t *row = table->get(r);
    if (row != NULL) {
      row->add((int32_t)0);
    }
  }
}

void tableRemoveRow(nairda_table_t *table) {
  if (table->size() == 0) return;
  nairda_row_t *row = table->pop();
  if (row != NULL) {
    row->clear();
    delete row;
  }
}

void tableRemoveColumn(nairda_table_t *table) {
  for (int r = 0; r < table->size(); r++) {
    nairda_row_t *row = table->get(r);
    if (row != NULL && row->size() > 0) {
      row->pop();
    }
  }
}
