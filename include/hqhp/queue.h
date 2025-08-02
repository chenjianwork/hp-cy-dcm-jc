/*!
****************************************************************************************************
* 文件名称：queue.h
* 功能简介：
* 文件作者：HQHP
* 创建日期：2023-01-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_QUEUE_H_
#define INCLUDE_HQHP_QUEUE_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <string.h>
#include <stdbool.h>
#include <hqhp/defs.h>
#ifdef __cplusplus
extern "C" {
#endif

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _queue {
	int16_t	 nums;
	int16_t	 head;
	int16_t	 tail;
	int16_t	 size;
	uint8_t* data;
};
typedef struct _queue queue_t;

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/
static inline void queue_init(queue_t* q, uint8_t* buf, size_t bufsize)
{
	q->nums = 0;
	q->head = 0;
	q->tail = 0;
	q->size = bufsize;
	q->data = buf;
}

static inline bool queue_is_empty(queue_t* q)
{
	return (bool)(q->nums == 0);
}

static inline bool queue_is_full(queue_t* q)
{
	return (bool)(q->nums == q->size);
}

static inline size_t queue_inq_nums(queue_t* q)
{
	return q->nums;
}

static inline size_t queue_inq_space(queue_t* q)
{
	return q->size - q->nums;
}

static inline bool queue_add_tail(queue_t* q, uint8_t data)
{
	if (q->nums < q->size) {
		q->nums++;
		q->data[q->tail++] = data;
		if (q->tail >= q->size) {
			q->tail = 0;
		}

		return true;
	}

	return false;
}

static inline uint8_t queue_dequeue(queue_t* q)
{
	uint8_t data;

	if (q->nums > 0) {
		data = q->data[q->head];
		q->head++;
		if (q->head >= q->size) {
			q->head = 0;
		}
		q->nums--;

		return data;
	}

	return 0;
}

static inline bool queue_del_tail(queue_t* q)
{
	if (q->nums > 0) {
		q->nums--;
		q->tail--;
		if (q->tail < 0) {
			q->tail = q->size - 1;
		}

		return true;
	}

	return false;
}

#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_HQHP_QUEUE_H_ */
