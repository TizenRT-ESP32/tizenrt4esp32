/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <string.h>
#include <semaphore.h>
#include <arastorage/arastorage.h>
#include <tinyara/fs/fs_utils.h>
#include "tc_common.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
#ifndef CONFIG_MOUNT_POINT
#define CONFIG_MOUNT_POINT "/mnt"
#endif

#define RELATION_NAME1  "rel1"
#define RELATION_NAME2  "rel2"
#define INDEX_BPLUS     "bplustree"
#define INDEX_INLINE    "inline"
#define QUERY_LENGTH    128

#define DATA_SET_NUM    10
#define DATA_SET_MULTIPLIER 80

/****************************************************************************
 *  Global Variables
 ****************************************************************************/
static db_cursor_t *g_cursor;

const static char *g_attribute_set[] = {"id", "date", "fruit", "value", "weight"};

struct arastorage_data_type_s {
	long long_value;
	char *string_value;
	double double_value;
};

const static struct arastorage_data_type_s g_arastorage_data_set[DATA_SET_NUM] = {
	{20160101,       "apple"     , 1.0 },
	{20160102,       "banana"    , 2.0 },
	{20160103,       "mango"     , 3.0 },
	{20160104,       "orange"    , 4.0 },
	{20160105,       "melon"     , 5.0 },
	{20160106,       "kiwi"      , 6.0 },
	{20160107,       "peach"     , 7.0 },
	{20160108,       "cherry"    , 8.0 },
	{20160109,       "strawberry", 9.0 },
	{20160110,       "watermelon", 10.0}
};

static void check_query_result(char * query)
{
	db_result_t res;
	g_cursor = db_query(query);
	TC_ASSERT_NEQ("db_query", g_cursor, NULL);
	if (DB_SUCCESS(cursor_move_first(g_cursor))) {
		do {
			res = db_print_tuple(g_cursor);
			TC_ASSERT_EQ("db_print_tuple", DB_SUCCESS(res), true);
		} while (DB_SUCCESS(cursor_move_next(g_cursor)));
	}

	res = db_cursor_free(g_cursor);
	TC_ASSERT_EQ("db_cursor_free", DB_SUCCESS(res), true);
	g_cursor = NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Clean up all components used in these TCs */
static void cleanup(void)
{
	char query[QUERY_LENGTH];

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "REMOVE RELATION %s;", RELATION_NAME1);
	db_exec(query);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "REMOVE RELATION %s;", RELATION_NAME2);
	db_exec(query);
}

/**
* @testcase         utc_arastorage_db_init_p
* @brief            Initialize database resources
* @scenario         Initialize resources used in database
* @apicovered       db_init
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_init_p(void)
{
	db_result_t res;

	res = db_init();
	TC_ASSERT_EQ("db_init", DB_SUCCESS(res), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_deinit_p
* @brief            De-initialize database resources
* @scenario         Release any resources used in database
* @apicovered       db_deinit
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_deinit_p(void)
{
	db_result_t res;

	res = db_deinit();
	TC_ASSERT_EQ("db_deinit", DB_SUCCESS(res), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_exec_p
* @brief            Excute database operations
* @scenario         Create relation, attributes, index and insert data
* @apicovered       db_exec
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_exec_p(void)
{
	db_result_t res;
	char query[QUERY_LENGTH];
	int i;

	/* cleanup called for cleaning all undeleted resources */
	cleanup();

	snprintf(query, QUERY_LENGTH, "CREATE RELATION %s;", RELATION_NAME1);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	snprintf(query, QUERY_LENGTH, "CREATE RELATION %s;", RELATION_NAME2);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN int IN %s;", g_attribute_set[0],
			 RELATION_NAME2);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN long IN %s;", g_attribute_set[1],
			 RELATION_NAME2);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE INDEX %s.%s TYPE %s;", RELATION_NAME2, g_attribute_set[1], INDEX_BPLUS);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	for (i = 0; i < DATA_SET_NUM * 10; i++) {
		memset(query, 0, QUERY_LENGTH);
		snprintf(query, QUERY_LENGTH, "INSERT (%d, %ld) INTO %s;", i, rand()%10000, RELATION_NAME2);
		res = db_exec(query);
		TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);
	}

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "REMOVE RELATION %s;", RELATION_NAME2);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE RELATION %s;", RELATION_NAME2);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN int IN %s;", g_attribute_set[0], RELATION_NAME1);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN long IN %s;", g_attribute_set[1], RELATION_NAME1);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN string(32) IN %s;", g_attribute_set[2], RELATION_NAME1);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN int IN %s;", g_attribute_set[3], RELATION_NAME1);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN int IN %s;", g_attribute_set[0], RELATION_NAME2);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN long IN %s;", g_attribute_set[1], RELATION_NAME2);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	sprintf(query, "CREATE INDEX %s.%s TYPE %s;", RELATION_NAME1, g_attribute_set[0], INDEX_INLINE);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	sprintf(query, "CREATE INDEX %s.%s TYPE %s;", RELATION_NAME2, g_attribute_set[1], INDEX_BPLUS);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);


	for (i = 0; i < DATA_SET_NUM * 10; i++) {
		memset(query, 0, QUERY_LENGTH);
		snprintf(query, QUERY_LENGTH, "INSERT (%d, %ld) INTO %s;", i, rand()%10000, RELATION_NAME2);
		res = db_exec(query);
		TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);
	}

#ifdef CONFIG_ARCH_FLOAT_H
	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN double IN %s;", g_attribute_set[4], RELATION_NAME1);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	for (i = 0; i < DATA_SET_NUM; i++) {
		memset(query, 0, QUERY_LENGTH);
		snprintf(query, "INSERT (%d, %ld, \'%s\', %d, %f) INTO %s;", QUERY_LENGTH,
				 i, g_arastorage_data_set[i].long_value,
				 g_arastorage_data_set[i].string_value, 1000 - i,
				 g_arastorage_data_set[i].double_value, RELATION_NAME1);
		res = db_exec(query);
		TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);
	}
#else
	for (i = 0; i < DATA_SET_NUM; i++) {
		memset(query, 0, QUERY_LENGTH);
		snprintf(query, QUERY_LENGTH, "INSERT (%d, %ld, \'%s\', %d) INTO %s;",
				 i, g_arastorage_data_set[i].long_value,
				 g_arastorage_data_set[i].string_value, 1000 - i, RELATION_NAME1);
		res = db_exec(query);
		TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);
	}
#endif
	memset(query, 0, QUERY_LENGTH);
	sprintf(query, "CREATE INDEX %s.%s TYPE %s;", RELATION_NAME1, g_attribute_set[3], INDEX_BPLUS);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);


#ifdef CONFIG_ARCH_FLOAT_H

	for (; i < DATA_SET_NUM * DATA_SET_MULTIPLIER; i++) {
		memset(query, 0, QUERY_LENGTH);
		snprintf(query, "INSERT (%d, %ld, \'%s\', %d, %f) INTO %s;", QUERY_LENGTH,
				 i, g_arastorage_data_set[i % DATA_SET_NUM].long_value,
				 g_arastorage_data_set[i % DATA_SET_NUM].string_value, 1000 - i,
				 g_arastorage_data_set[i % DATA_SET_NUM].double_value, RELATION_NAME1);
		res = db_exec(query);
		TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);
	}
#else
	for (; i < DATA_SET_NUM * DATA_SET_MULTIPLIER; i++) {
		memset(query, 0, QUERY_LENGTH);
		snprintf(query, QUERY_LENGTH, "INSERT (%d, %ld, \'%s\', %d) INTO %s;",
				 i, g_arastorage_data_set[i % DATA_SET_NUM].long_value,
				 g_arastorage_data_set[i % DATA_SET_NUM].string_value, 1000 - i, RELATION_NAME1);
		res = db_exec(query);
		TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);
	}
#endif

	memset(query, 0, QUERY_LENGTH);
	sprintf(query, "REMOVE INDEX %s.%s ;", RELATION_NAME1, g_attribute_set[0]);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	memset(query, 0, QUERY_LENGTH);
	sprintf(query, "CREATE INDEX %s.%s TYPE %s;", RELATION_NAME1, g_attribute_set[0], INDEX_INLINE);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_SUCCESS(res), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_exec_n
* @brief            Excute database operations with invalid argument
* @scenario         Create exiting relation, attribute and excute db with NULL value
* @apicovered       db_exec
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_exec_n(void)
{
	db_result_t res;
	char query[QUERY_LENGTH];
	char *name = "BAD_RELATION";
	char *attr_name = "attr";
	/* Try to create existing relation */
	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE RELATION %s;", RELATION_NAME1);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_ERROR(res), true);

	/* Try to create existing attribute */
	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN int IN %s;", g_attribute_set[0], RELATION_NAME1);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_ERROR(res), true);

	/* Try to excute db with NULL value */
	res = db_exec(NULL);
	TC_ASSERT_EQ("db_exec", DB_ERROR(res), true);

	/* Try to parse mis-spelt query */
	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CRAETE ATTRIBUTE %s DOMAIN int IN %s;", g_attribute_set[0], RELATION_NAME1);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_ERROR(res), true);

	/* Create duplicate attribute */
	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE ATTRIBUTE %s DOMAIN int IN %s;", g_attribute_set[0], name);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_ERROR(res), true);

	/* Index on non-existent attribute */
	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "CREATE INDEX %s.%s TYPE %s;", RELATION_NAME1, attr_name, INDEX_BPLUS);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_ERROR(res), true);

	/* execute un-implemented operation */
	memset(query, 0, QUERY_LENGTH);
	snprintf(query, QUERY_LENGTH, "REMOVE ATTRIBUTE %s.%s;", RELATION_NAME1, g_attribute_set[0]);
	res = db_exec(query);
	TC_ASSERT_EQ("db_exec", DB_ERROR(res), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_query_p
* @brief            Query a database
* @scenario         Select data and get returned cursor
* @apicovered       db_query
* @precondition     utc_arastorage_db_exec_p should be passed
* @postcondition    none
*/
static void utc_arastorage_db_query_p(void)
{
	db_result_t res;
	char query[QUERY_LENGTH];

	/* Select over bplus-tree index */
#ifdef CONFIG_ARCH_FLOAT_H
	snprintf(query, QUERY_LENGTH, "SELECT id, date, fruit, value, weight FROM %s WHERE value < 990;", RELATION_NAME1);

#else
	snprintf(query, QUERY_LENGTH, "SELECT id, date, fruit, value FROM %s WHERE value < 990;", RELATION_NAME1);
#endif
	g_cursor = db_query(query);
	TC_ASSERT_NEQ("db_query", g_cursor, NULL);

	res = db_cursor_free(g_cursor);
	TC_ASSERT_EQ("db_cursor_free", DB_SUCCESS(res), true);
	g_cursor = NULL;

	/* Select with AND condition */
	snprintf(query, QUERY_LENGTH, "SELECT id, date FROM %s WHERE id < 25 AND id > 10;", RELATION_NAME2);
	check_query_result(query);

	/* Count aggregation operation */
	snprintf(query, QUERY_LENGTH, "SELECT COUNT(id) FROM %s WHERE id = 5 OR id = 85;", RELATION_NAME2);
	check_query_result(query);

	/* Mean aggregation operation */
	snprintf(query, QUERY_LENGTH, "SELECT MEAN(id) FROM %s;", RELATION_NAME2);
	check_query_result(query);

	/* Sum aggregation operation */
	snprintf(query, QUERY_LENGTH, "SELECT SUM(id) FROM %s;", RELATION_NAME2);
	check_query_result(query);

	/* Max aggregation operation */
	snprintf(query, QUERY_LENGTH, "SELECT MAX(id) FROM %s;", RELATION_NAME2);
	check_query_result(query);

	/* Min aggregation operation */
	snprintf(query, QUERY_LENGTH, "SELECT MIN(id) FROM %s;", RELATION_NAME2);
	check_query_result(query);

	/* Remove operation */
	snprintf(query, QUERY_LENGTH, "REMOVE FROM %s WHERE date > 2000 AND date < 8000;", RELATION_NAME2);
	check_query_result(query);

	/* Select over inline index */

#ifdef CONFIG_ARCH_FLOAT_H
	snprintf(query, QUERY_LENGTH, "SELECT id, date, fruit, weight FROM %s WHERE id > 0;", RELATION_NAME1);

#else
	snprintf(query, QUERY_LENGTH, "SELECT id, date, fruit FROM %s WHERE id > 0;", RELATION_NAME1);
#endif
	g_cursor = db_query(query);
	TC_ASSERT_NEQ("db_query", g_cursor, NULL);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_query_n
* @brief            Query a database with invalid argument
* @scenario         Select data from invalid relation and query db with NULL value
* @apicovered       db_query
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_query_n(void)
{
	char query[QUERY_LENGTH];
	char *name = "BAD_RELATION";

	snprintf(query, QUERY_LENGTH, "SELECT %s, %s, %s FROM %s WHERE %s > 5;", g_attribute_set[0],
			 g_attribute_set[1], g_attribute_set[2], name, g_attribute_set[0]);
	g_cursor = db_query(query);
	TC_ASSERT_EQ("db_query", g_cursor, NULL);

	g_cursor = db_query(NULL);
	TC_ASSERT_EQ("db_query", g_cursor, NULL);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_get_result_message_p
* @brief            Get database result message
* @scenario         Get database result message corresponding to result value
* @apicovered       db_get_result_message
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_get_result_message_p(void)
{
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_FINISHED_MSG, db_get_result_message(DB_FINISHED)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_OK_MSG, db_get_result_message(DB_OK)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_LIMIT_ERROR_MSG, db_get_result_message(DB_LIMIT_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_ALLOCATION_ERROR_MSG, db_get_result_message(DB_ALLOCATION_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_STORAGE_ERROR_MSG, db_get_result_message(DB_STORAGE_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_PARSING_ERROR_MSG, db_get_result_message(DB_PARSING_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_NAME_ERROR_MSG, db_get_result_message(DB_NAME_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_RELATIONAL_ERROR_MSG, db_get_result_message(DB_RELATIONAL_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_TYPE_ERROR_MSG, db_get_result_message(DB_TYPE_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_IMPLEMENTATION_ERROR_MSG, db_get_result_message(DB_IMPLEMENTATION_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_INDEX_ERROR_MSG, db_get_result_message(DB_INDEX_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_BUSY_ERROR_MSG, db_get_result_message(DB_BUSY_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_INCONSISTENCY_ERROR_MSG, db_get_result_message(DB_INCONSISTENCY_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_ARGUMENT_ERROR_MSG, db_get_result_message(DB_ARGUMENT_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_FULL_ERROR_MSG, db_get_result_message(DB_FULL_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_CURSOR_ERROR_MSG, db_get_result_message(DB_CURSOR_ERROR)), 0);
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_EMPTY_CURSOR_ERROR_MSG, db_get_result_message(DB_EMPTY_CURSOR_ERROR)), 0);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_get_result_message_n
* @brief            Get database result message with invalid argument
* @scenario         Get database result message with out of range result value
* @apicovered       db_get_result_message
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_get_result_message_n(void)
{
	int num;

	srand(18);
	num = rand() - 15;
	TC_ASSERT_EQ("db_get_result_message", strcmp(DB_UNKNOWN_ERROR_MSG, db_get_result_message(num)), 0);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_print_header_p
* @brief            Print a header of query result
* @scenario         Print relation, attribute name of query result
* @apicovered       db_print_header
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_db_print_header_p(void)
{
	db_result_t res;

	res = db_print_header(g_cursor);
	TC_ASSERT_EQ("db_print_header", DB_SUCCESS(res), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_print_header_n
* @brief            Print a header of query result with invalid argument
* @scenario         Print relation, attribute name of query result with NULL value
* @apicovered       db_print_header
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_print_header_n(void)
{
	db_result_t res;

	res = db_print_header(NULL);
	TC_ASSERT_EQ("db_print_header", DB_ERROR(res), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_print_tuple_p
* @brief            Print tuple pointed by cursor
* @scenario         Print tuple values from the first to the last row
* @apicovered       cursor_move_first, db_print_tuple, cursor_move_next
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_db_print_tuple_p(void)
{
	db_result_t res;

	if (DB_SUCCESS(cursor_move_first(g_cursor))) {
		do {
			res = db_print_tuple(g_cursor);
			TC_ASSERT_EQ("db_print_tuple", DB_SUCCESS(res), true);
		} while (DB_SUCCESS(cursor_move_next(g_cursor)));
	}

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_print_tuple_n
* @brief            Print tuple pointed by cursor with invalid argument
* @scenario         Print tuple values from the first to the last row with NULL value
* @apicovered       cursor_move_first, db_print_tuple, cursor_move_next
* @precondition	    none
* @postcondition    none
*/
static void utc_arastorage_db_print_tuple_n(void)
{
	db_result_t res;

	res = db_print_tuple(NULL);
	TC_ASSERT_EQ("db_print_tuple", DB_ERROR(res), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_print_value_p
* @brief            Print a attribute value of tuple pointed by cursor
* @scenario         Print a specific attribute value of tuple pointed by cursor
* @apicovered       db_print_value
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_db_print_value_p(void)
{
	db_result_t res;

	res = db_print_value(g_cursor, 0);
	TC_ASSERT_EQ("db_print_value", DB_SUCCESS(res), true);

	res = db_print_value(g_cursor, 1);
	TC_ASSERT_EQ("db_print_value", DB_SUCCESS(res), true);

	res = db_print_value(g_cursor, 2);
	TC_ASSERT_EQ("db_print_value", DB_SUCCESS(res), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_print_value_n
* @brief            Print a attribute value of tuple pointed by cursor with invalid argument
* @scenario         Print a specific attribute value of tuple pointed by cursor with NULL value
* @apicovered       db_print_value
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_print_value_n(void)
{
	db_result_t res;

	res = db_print_value(NULL, 2);
	TC_ASSERT_EQ("db_print_value", DB_ERROR(res), true);

	res = db_print_value(g_cursor, 5);
	TC_ASSERT_EQ("db_print_value", DB_ERROR(res), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_cursor_free_p
* @brief            Free cursor
* @scenario         Free allocated resources for cursor
* @apicovered       db_cursor_free
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_db_cursor_free_p(void)
{
	db_result_t res;

	res = db_cursor_free(g_cursor);
	TC_ASSERT_EQ("db_cursor_free", DB_SUCCESS(res), true);
	g_cursor = NULL;

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_db_cursor_free_n
* @brief            Free cursor with invalid argument
* @scenario         Free allocated resources for cursor with NULL value
* @apicovered       db_cursor_free
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_db_cursor_free_n(void)
{
	db_result_t res;

	res = db_cursor_free(NULL);
	TC_ASSERT_EQ("db_cursor_free", DB_ERROR(res), true);

	/* free cursor properly here to prevent memory leak */
	res = db_cursor_free(g_cursor);
	TC_ASSERT_EQ("db_cursor_free", DB_SUCCESS(res), true);
	g_cursor = NULL;

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_count_p
* @brief            Get the number of tuples in cursor
* @scenario         Get the number of tuples in cursor
* @apicovered       cursor_get_count
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_get_count_p(void)
{
	tuple_id_t count;

	count = cursor_get_count(g_cursor);
	TC_ASSERT_GT("cursor_get_count", count, 0);
	printf("cursor count : %d\n", count);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_count_n
* @brief            Get the number of tuples in cursor with invalid argument
* @scenario         Get the number of tuples in cursor with NULL value
* @apicovered       cursor_get_count
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_get_count_n(void)
{
	tuple_id_t count;

	count = cursor_get_count(NULL);
	TC_ASSERT_EQ("cursor_get_count", count, INVALID_CURSOR_VALUE);
	printf("cursor count : %d\n", count);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_first_p
* @brief            Move cursor to the first row
* @scenario         Move cursor to the first row and check whether current row is set as the first row
* @apicovered       cursor_move_first, cursor_get_row
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_move_first_p(void)
{
	db_result_t res;

	res = cursor_move_first(g_cursor);
	TC_ASSERT_EQ("cursor_move_first", DB_SUCCESS(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));
	TC_ASSERT_EQ("cursor_get_row", cursor_get_row(g_cursor), 0);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_first_n
* @brief            Move cursor to the first row with invalid argument
* @scenario         Move cursor to the first row with NULL value
* @apicovered       cursor_move_first, cursor_get_row
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_move_first_n(void)
{
	db_result_t res;

	res = cursor_move_first(NULL);
	TC_ASSERT_EQ("cursor_move_first", DB_ERROR(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_next_p
* @brief            Move cursor to next row
* @scenario         Move cursor to next row and check whether current row is set as the next row
* @apicovered       cursor_move_next, cursor_get_row
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_move_next_p(void)
{
	db_result_t res;

	res = cursor_move_next(g_cursor);
	TC_ASSERT_EQ("cursor_move_next", DB_SUCCESS(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));
	TC_ASSERT_EQ("cursor_get_row", cursor_get_row(g_cursor), 1);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_next_n
* @brief            Move cursor to next row with invalid argument
* @scenario         Move cursor to next row with NULL value
* @apicovered       cursor_move_next, cursor_get_row
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_move_next_n(void)
{
	db_result_t res;

	res = cursor_move_next(NULL);
	TC_ASSERT_EQ("cursor_move_next", DB_ERROR(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_prev_p
* @brief            Move cursor to previous row
* @scenario         Move cursor to previous row and checkwhether current row is set as the previous row
* @apicovered       cursor_move_next, cursor_get_row
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_move_prev_p(void)
{
	db_result_t res;

	res = cursor_move_prev(g_cursor);
	TC_ASSERT_EQ("cursor_move_prev", DB_SUCCESS(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));
	TC_ASSERT_EQ("cursor_get_row", cursor_get_row(g_cursor), 0);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_prev_n
* @brief            Move cursor to previous row with invalid argument
* @scenario         Move cursor to previous row with NULL value
* @apicovered       cursor_move_prev, cursor_get_row
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_move_prev_n(void)
{
	db_result_t res;

	res = cursor_move_prev(NULL);
	TC_ASSERT_EQ("cursor_move_prev", DB_ERROR(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_last_p
* @brief            Move cursor to the last row
* @scenario         Move cursor to the last row and check whether current row is set as the last row
* @apicovered       cursor_move_last, cursor_get_row
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_move_last_p(void)
{
	db_result_t res;

	res = cursor_move_last(g_cursor);
	TC_ASSERT_EQ("cursor_move_last", DB_SUCCESS(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));
	TC_ASSERT_EQ("cursor_get_row", cursor_get_row(g_cursor), DATA_SET_NUM * DATA_SET_MULTIPLIER - 2);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_last_n
* @brief            Move cursor to the last row with invalid argument
* @scenario         Move cursor to the last row with NULL value
* @apicovered       cursor_move_last, cursor_get_row
* @precondition	    none
* @postcondition    none
*/
static void utc_arastorage_cursor_move_last_n(void)
{
	db_result_t res;

	res = cursor_move_last(NULL);
	TC_ASSERT_EQ("cursor_move_last", DB_ERROR(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_last_p
* @brief            Move cursor to specific row
* @scenario         Move cursor to specific row and check whether current row is set as the specific row
* @apicovered       cursor_move_to, cursor_get_row
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_move_to_p(void)
{
	db_result_t res;
	tuple_id_t row = 3;

	res = cursor_move_to(g_cursor, row);
	TC_ASSERT_EQ("cursor_move_to", DB_SUCCESS(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));
	TC_ASSERT_EQ("cursor_get_row", cursor_get_row(g_cursor), row);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_move_last_n
* @brief            Move cursor to specific row with invalid argument
* @scenario         Move cursor to out of range row value and valid row value with NULL cursor value
* @apicovered       cursor_move_to, cursor_get_row
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_move_to_n(void)
{
	db_result_t res;
	tuple_id_t row = 3;

	res = cursor_move_to(g_cursor, -1);
	TC_ASSERT_EQ("cursor_move_to", DB_ERROR(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));
	TC_ASSERT_NEQ("cursor_get_row", cursor_get_row(g_cursor), row);

	res = cursor_move_to(NULL, 0);
	TC_ASSERT_EQ("cursor_move_to", DB_ERROR(res), true);
	printf("cursor row = %d\n", cursor_get_row(g_cursor));

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_row_p
* @brief            Get current row in cursor
* @scenario         Get current row in cursor
* @apicovered       cursor_get_row
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_get_row_p(void)
{
	cursor_row_t row;
	tuple_id_t id = 3;

	row = cursor_get_row(g_cursor);
	TC_ASSERT_EQ("cursor_get_row", row, id);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_row_n
* @brief            Get current row in cursor with invalid argument
* @scenario         Get current row in cursor with NULL value
* @apicovered       cursor_get_row
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_get_row_n(void)
{
	cursor_row_t row;
	tuple_id_t id = 3;

	row = cursor_get_row(g_cursor);
	TC_ASSERT_NEQ("cursor_get_row", row, id);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_is_first_row_p
* @brief            Check whether cursor points the first row
* @scenario         Move cursor to the first row and check whether cursor points the first row
* @apicovered       cursor_move_first, cursor_is_first_row
* @precondition	    utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_is_first_row_p(void)
{
	db_result_t res;

	res = cursor_move_first(g_cursor);
	TC_ASSERT_EQ("cursor_move_first", DB_SUCCESS(res), true);
	TC_ASSERT_EQ("cursor_is_first_row", cursor_is_first_row(g_cursor), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_is_first_row_n
* @brief            Check whether cursor points the first row with invalid argument
* @scenario         Move cursor to the last row and check whether current row is set as the first row,
*                   Check whether cursor points the first row with NULL value
* @apicovered       db_query, cursor_move_last, cursor_is_first_row
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_is_first_row_n(void)
{
	db_result_t res;
	char query[QUERY_LENGTH];

	/* We need to get valid cursor for this test */
	snprintf(query, QUERY_LENGTH, "SELECT %s, %s, %s FROM %s WHERE %s > 0;", g_attribute_set[0],
			 g_attribute_set[1], g_attribute_set[2], RELATION_NAME1, g_attribute_set[0]);
	g_cursor = db_query(query);
	TC_ASSERT_NEQ("db_query", g_cursor, NULL);
	res = cursor_move_last(g_cursor);
	TC_ASSERT_EQ("cursor_move_last", DB_SUCCESS(res), true);
	/* Cursor is valid, but it isn't pointing first row */
	TC_ASSERT_NEQ("cursor_is_first_row", cursor_is_first_row(g_cursor), true);
	/* Invalid argument */
	TC_ASSERT_NEQ("cursor_is_first_row", cursor_is_first_row(NULL), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_is_last_row_p
* @brief            Check whether cursor points the last row
* @scenario         Move cursor to the last row and check whether cursor points the last row
* @apicovered       cursor_move_last, cursor_is_last_row
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_is_last_row_p(void)
{
	db_result_t res;

	res = cursor_move_last(g_cursor);
	TC_ASSERT_EQ("cursor_move_last", DB_SUCCESS(res), true);
	TC_ASSERT_EQ("cursor_is_last_row", cursor_is_last_row(g_cursor), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_is_last_row_n
* @brief            Check whether cursor points the last row with invalid argument
* @scenario         Move cursor to the first row and check whether current row is set as the last row,
*                   Check whether cursor points the last row with NULL value
* @apicovered       db_query, cursor_move_first, cursor_is_last_row
* @precondition	    none
* @postcondition    none
*/
static void utc_arastorage_cursor_is_last_row_n(void)
{
	db_result_t res;
	char query[QUERY_LENGTH];

	/* We need to get valid cursor for this test */
	snprintf(query, QUERY_LENGTH, "SELECT %s, %s, %s FROM %s WHERE %s > 0;", g_attribute_set[0],
			 g_attribute_set[1], g_attribute_set[2], RELATION_NAME1, g_attribute_set[0]);
	g_cursor = db_query(query);
	TC_ASSERT_NEQ("db_query", g_cursor, NULL);
	res = cursor_move_first(g_cursor);
	TC_ASSERT_EQ("cursor_move_first", DB_SUCCESS(res), true);
	/* Cursor is valid, but it isn't pointing last row */
	TC_ASSERT_NEQ("cursor_is_last_row", cursor_is_last_row(g_cursor), true);
	/* Invalid argument */
	TC_ASSERT_NEQ("cursor_is_last_row", cursor_is_last_row(NULL), true);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_attr_type_p
* @brief            Get a domain(type) of specific attribute
* @scenario         Get a domain of specific attribute
* @apicovered       cursor_get_attr_type
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_get_attr_type_p(void)
{
	domain_t domain;

	domain = cursor_get_attr_type(g_cursor, 0);
	TC_ASSERT_EQ("cursor_get_attr_type", domain, DOMAIN_INT);

	domain = cursor_get_attr_type(g_cursor, 1);
	TC_ASSERT_EQ("cursor_get_attr_type", domain, DOMAIN_LONG);

	domain = cursor_get_attr_type(g_cursor, 2);
	TC_ASSERT_EQ("cursor_get_attr_type", domain, DOMAIN_STRING);

#ifdef CONFIG_ARCH_FLOAT_H
	domain = cursor_get_attr_type(g_cursor, 3);
	TC_ASSERT_EQ("cursor_get_attr_type", domain, DOMAIN_DOUBLE);

#endif
	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_attr_type_n
* @brief            Get a domain(type) of specific attribute with invalid argument
* @scenario         Get a domain of specific attribute with NULL value
*                   Get a domain of attribute with out of range attribute index
* @apicovered       cursor_get_attr_type
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_get_attr_type_n(void)
{
	domain_t domain;

	domain = cursor_get_attr_type(NULL, 0);
	TC_ASSERT_NEQ("cursor_get_attr_type", domain, DOMAIN_INT);
	TC_ASSERT_NEQ("cursor_get_attr_type", domain, DOMAIN_LONG);
	TC_ASSERT_NEQ("cursor_get_attr_type", domain, DOMAIN_STRING);
#ifdef CONFIG_ARCH_FLOAT_H
	TC_ASSERT_NEQ("cursor_get_attr_type", domain, DOMAIN_DOUBLE);
#endif

	domain = cursor_get_attr_type(g_cursor, 6);
	TC_ASSERT_NEQ("cursor_get_attr_type", domain, DOMAIN_INT);
	TC_ASSERT_NEQ("cursor_get_attr_type", domain, DOMAIN_LONG);
	TC_ASSERT_NEQ("cursor_get_attr_type", domain, DOMAIN_STRING);
#ifdef CONFIG_ARCH_FLOAT_H
	TC_ASSERT_NEQ("cursor_get_attr_type", domain, DOMAIN_DOUBLE);
#endif

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_attr_name_p
* @brief            Get a name of specific attribute
* @scenario         Get a name of specific attribute
* @apicovered       cursor_get_attr_name
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_get_attr_name_p(void)
{
	char *attr_name;

	attr_name = cursor_get_attr_name(g_cursor, 0);
	printf("attr name index 0 : %s\n", attr_name);
	TC_ASSERT_EQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[0]), 0);

	attr_name = cursor_get_attr_name(g_cursor, 1);
	printf("attr name index 1 : %s\n", attr_name);
	TC_ASSERT_EQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[1]), 0);

	attr_name = cursor_get_attr_name(g_cursor, 2);
	printf("attr name index 2 : %s\n", attr_name);
	TC_ASSERT_EQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[2]), 0);

#ifdef CONFIG_ARCH_FLOAT_H
	attr_name = cursor_get_attr_name(g_cursor, 3);
	printf("attr name index 3 : %s\n", attr_name);
	TC_ASSERT_EQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[3]), 0);
#endif

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_attr_name_n
* @brief            Get a name of specific attribute with invalid argument
* @scenario         Get a name of specific attribute with NULL value
*                   Get a name of attribute with out of range attribute index
* @apicovered       cursor_get_attr_name
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_get_attr_name_n(void)
{
	char *attr_name;

	attr_name = cursor_get_attr_name(NULL, 0);
	printf("attr name index 0 : %s\n", attr_name);
	TC_ASSERT_NEQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[0]), 0);

	attr_name = cursor_get_attr_name(NULL, 1);
	printf("attr name index 1 : %s\n", attr_name);
	TC_ASSERT_NEQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[1]), 0);

	attr_name = cursor_get_attr_name(NULL, 2);
	printf("attr name index 2 : %s\n", attr_name);
	TC_ASSERT_NEQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[2]), 0);

	attr_name = cursor_get_attr_name(g_cursor, 3);
	printf("attr name index 3 : %s\n", attr_name);
	TC_ASSERT_NEQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[3]), 0);

#ifdef CONFIG_ARCH_FLOAT_H
	attr_name = cursor_get_attr_name(NULL, 3);
	printf("attr name index 3 : %s\n", attr_name);
	TC_ASSERT_NEQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[3]), 0);

	attr_name = cursor_get_attr_name(g_cursor, 4);
	printf("attr name index 3 : %s\n", attr_name);
	TC_ASSERT_NEQ("cursor_get_attr_name", strcmp((const char *)attr_name, g_attribute_set[3]), 0);
#endif

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_attr_index_p
* @brief            Get a index of specific attribute
* @scenario         Get a index of specific attribute
* @apicovered       cursor_get_attr_index
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_get_attr_index_p(void)
{
	attribute_id_t index;

	index = cursor_get_attr_index(g_cursor, g_attribute_set[0]);
	printf("attr name %s index : %d\n", g_attribute_set[0], index);
	TC_ASSERT_EQ("cursor_get_attr_index", index, 0);

	index = cursor_get_attr_index(g_cursor, g_attribute_set[1]);
	printf("attr name %s index : %d\n", g_attribute_set[1], index);
	TC_ASSERT_EQ("cursor_get_attr_index", index, 1);

	index = cursor_get_attr_index(g_cursor, g_attribute_set[2]);
	printf("attr name %s index : %d\n", g_attribute_set[2], index);
	TC_ASSERT_EQ("cursor_get_attr_index", index, 2);

#ifdef CONFIG_ARCH_FLOAT_H
	index = cursor_get_attr_index(g_cursor, g_attribute_set[3]);
	printf("attr name %s index : %d\n", g_attribute_set[3], index);
	TC_ASSERT_EQ("cursor_get_attr_index", index, INVALID_CURSOR_VALUE);
#endif

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_attr_index_n
* @brief            Get a index of specific attribute with invalid argument
* @scenario         Get a index of specific attribute with NULL value
* @apicovered       cursor_get_attr_index
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_get_attr_index_n(void)
{
	attribute_id_t index;

	index = cursor_get_attr_index(NULL, g_attribute_set[0]);
	printf("attr name %s index : %d\n", g_attribute_set[0], index);
	TC_ASSERT_NEQ("cursor_get_attr_index", index, 0);

	index = cursor_get_attr_index(NULL, g_attribute_set[1]);
	printf("attr name %s index : %d\n", g_attribute_set[1], index);
	TC_ASSERT_NEQ("cursor_get_attr_index", index, 1);

	index = cursor_get_attr_index(NULL, g_attribute_set[2]);
	printf("attr name %s index : %d\n", g_attribute_set[2], index);
	TC_ASSERT_NEQ("cursor_get_attr_index", index, 2);

#ifdef CONFIG_ARCH_FLOAT_H
	index = cursor_get_attr_index(NULL, g_attribute_set[3]);
	printf("attr name %s index : %d\n", g_attribute_set[3], index);
	TC_ASSERT_NEQ("cursor_get_attr_index", index, 3);
#endif

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_int_value_p
* @brief            Get a int value of specific attribute
* @scenario         Get a int value of specific attribute
* @apicovered       cursor_get_int_value
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_get_int_value_p(void)
{
	int value;

	value = cursor_get_int_value(g_cursor, 0);
	TC_ASSERT_GEQ("cursor_get_int_value", value, 0);
	printf("value = %d\n", value);

	value = cursor_get_int_value(g_cursor, 1);
	TC_ASSERT_LT("cursor_get_int_value", value, 0);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_int_value_n
* @brief            Get a int value of specific attribute with invalid argument
* @scenario         Get a int value of specific attribute with NULL value
*                   Get a int value of attribute which the type is not integer
* @apicovered       cursor_get_int_value
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_get_int_value_n(void)
{
	int value;

	value = cursor_get_int_value(NULL, 0);
	TC_ASSERT_LT("cursor_get_int_value", value, 0);
	printf("value = %d\n", value);

	value = cursor_get_int_value(g_cursor, 1);
	TC_ASSERT_LT("cursor_get_int_value", value, 0);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_long_value_p
* @brief            Get a long value of specific attribute
* @scenario         Get a long value of specific attribute
* @apicovered       cursor_get_long_value
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_get_long_value_p(void)
{
	long value;

	value = cursor_get_long_value(g_cursor, 1);
	TC_ASSERT_GEQ("cursor_get_long_value", value, 0);
	printf("value = %ld\n", value);

	value = cursor_get_long_value(g_cursor, 2);
	TC_ASSERT_LT("cursor_get_long_value", value, 0);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_long_value_n
* @brief            Get a long value of specific attribute with invalid argument
* @scenario         Get a long value of specific attribute with NULL value
*                   Get a long value of attribute which the type is not long
* @apicovered       cursor_get_long_value
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_get_long_value_n(void)
{
	long value;

	value = cursor_get_long_value(NULL, 1);
	TC_ASSERT_LT("cursor_get_long_value", value, 0);
	printf("value = %ld\n", value);

	value = cursor_get_long_value(g_cursor, 2);
	TC_ASSERT_LT("cursor_get_long_value", value, 0);

	TC_SUCCESS_RESULT();
}

#ifdef CONFIG_ARCH_FLOAT_H
/**
* @testcase         utc_arastorage_cursor_get_double_value_p
* @brief            Get a double value of specific attribute
* @scenario         Get a double value of specific attribute
* @apicovered       cursor_get_double_value
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_get_double_value_p(void)
{
	double value;

	value = cursor_get_double_value(g_cursor, 3);
	TC_ASSERT_GEQ("cursor_get_double_value", value, 0);
	printf("value = %f\n", value);

	value = cursor_get_double_value(g_cursor, 2);
	TC_ASSERT_LT("cursor_get_double_value", value, 0);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_double_value_n
* @brief            Get a double value of specific attribute with invalid argument
* @scenario         Get a double value of specific attribute with NULL value
*                   Get a double value of attribute which the type is not double
* @apicovered       cursor_get_double_value
* @precondition     none
* @postcondition    none
*/
static void utc_arastorage_cursor_get_double_value_n(void)
{
	double value;

	value = cursor_get_double_value(NULL, 3);
	TC_ASSERT_LT("cursor_get_double_value", value, 0);
	printf("value = %f\n", value);

	value = cursor_get_double_value(g_cursor, 2);
	TC_ASSERT_LT("cursor_get_double_value", value, 0);

	TC_SUCCESS_RESULT();
}
#endif

/**
* @testcase         utc_arastorage_cursor_get_string_value_p
* @brief            Get a string value of specific attribute
* @scenario         Get a string value of specific attribute
* @apicovered       cursor_get_string_value
* @precondition     utc_arastorage_db_exec_p, utc_arastorage_db_query_p should be passed
* @postcondition    none
*/
static void utc_arastorage_cursor_get_string_value_p(void)
{
	unsigned char *value;

	value = cursor_get_string_value(g_cursor, 2);
	TC_ASSERT_NEQ("cursor_get_string_value", value, NULL);
	printf("value = %s\n", value);

	value = cursor_get_string_value(g_cursor, 0);
	TC_ASSERT_EQ("cursor_get_string_value", value, NULL);

	TC_SUCCESS_RESULT();
}

/**
* @testcase         utc_arastorage_cursor_get_string_value_n
* @brief            Get a string value of specific attribute with invalid argument
* @scenario         Get a string value of specific attribute with NULL value
*                   Get a string value of attribute which the type is not string
* @apicovered       cursor_get_string_value
* @precondition	    none
* @postcondition    none
*/
static void utc_arastorage_cursor_get_string_value_n(void)
{
	unsigned char *value;

	value = cursor_get_string_value(NULL, 2);
	TC_ASSERT_EQ("cursor_get_string_value", value, NULL);
	printf("value = %s\n", value);

	value = cursor_get_string_value(g_cursor, 0);
	TC_ASSERT_EQ("cursor_get_string_value", value, NULL);

	TC_SUCCESS_RESULT();
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int utc_arastorage_main(int argc, char *argv[])
#endif
{
	if (tc_handler(TC_START, "Arastorage UTC") == ERROR) {
		return ERROR;
	}

	/* Positive TCs */
	utc_arastorage_db_init_p();
	utc_arastorage_db_exec_p();
	utc_arastorage_db_query_p();
	utc_arastorage_db_get_result_message_p();
	utc_arastorage_db_print_header_p();
	utc_arastorage_db_print_tuple_p();
	utc_arastorage_db_print_value_p();
	utc_arastorage_cursor_get_count_p();
	utc_arastorage_cursor_move_first_p();
	utc_arastorage_cursor_move_next_p();
	utc_arastorage_cursor_move_prev_p();
	utc_arastorage_cursor_move_last_p();
	utc_arastorage_cursor_move_to_p();
	utc_arastorage_cursor_get_row_p();
	utc_arastorage_cursor_is_first_row_p();
	utc_arastorage_cursor_is_last_row_p();
	utc_arastorage_cursor_get_attr_type_p();
	utc_arastorage_cursor_get_attr_name_p();
	utc_arastorage_cursor_get_attr_index_p();
	utc_arastorage_cursor_get_int_value_p();
	utc_arastorage_cursor_get_long_value_p();
#ifdef CONFIG_ARCH_FLOAT_H
	utc_arastorage_cursor_get_double_value_p();
#endif
	utc_arastorage_cursor_get_string_value_p();
	utc_arastorage_db_cursor_free_p();
	utc_arastorage_db_deinit_p();

	db_init();

	/* Negative TCs */
	utc_arastorage_db_exec_n();
	utc_arastorage_db_query_n();
	utc_arastorage_db_get_result_message_n();
	utc_arastorage_db_print_header_n();
	utc_arastorage_db_print_tuple_n();
	utc_arastorage_db_print_value_n();
	utc_arastorage_cursor_get_count_n();
	utc_arastorage_cursor_move_first_n();
	utc_arastorage_cursor_move_next_n();
	utc_arastorage_cursor_move_prev_n();
	utc_arastorage_cursor_move_last_n();
	utc_arastorage_cursor_move_to_n();
	utc_arastorage_cursor_get_row_n();
	utc_arastorage_cursor_is_first_row_n();
	utc_arastorage_cursor_is_last_row_n();
	utc_arastorage_cursor_get_attr_type_n();
	utc_arastorage_cursor_get_attr_name_n();
	utc_arastorage_cursor_get_attr_index_n();
	utc_arastorage_cursor_get_int_value_n();
	utc_arastorage_cursor_get_long_value_n();
#ifdef CONFIG_ARCH_FLOAT_H
	utc_arastorage_cursor_get_double_value_n();
#endif
	utc_arastorage_cursor_get_string_value_n();
	utc_arastorage_db_cursor_free_n();
	cleanup();
	db_deinit();

	(void)tc_handler(TC_END, "Arastorage UTC");

	return 0;
}
