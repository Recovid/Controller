#ifndef IFL_DEQUE_H
#define IFL_DEQUE_H

// @Author: Inventhys
// @Date:   2020-04-08 17:14:01
// @Last Modified by:   Inventhys
// @Last Modified time: 2020-04-08 17:14:01

// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------

#include <stdint.h>

// --------------------------------------------------------------------------------------------------------------------
// ----- public constant macros
// --------------------------------------------------------------------------------------------------------------------
/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  Core
    @param[in]      array      Array to look at.
    @return   Implementation dependent (@ref ifl_hal_gpio_result_t).
    @brief    Returns the number of items in the given array.
*/
#define IFL_ARRAY_SIZE(array)   (sizeof(array) / sizeof((array)[0]) + _array_size_chk(array))

#if HAVE_BUILTIN_TYPES_COMPATIBLE_P && HAVE_TYPEOF
    // Two GCC extensions
    // &a[0] degrades to a pointer: a different type from an array
    #define _array_size_chk(arr) \
            BUILD_ASSERT_OR_ZERO(!__builtin_types_compatible_p(typeof(arr),  typeof(&(arr)[0])))
#else
    #define _array_size_chk(arr)    0
#endif

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to declare a deque.
    @param[in] name         Name of the deque.
    @param[in] type         Type of item contained in the deque.
    @param[in] size         Number of items that the deque can contain.
 **/
#define IFL_DEQUE_DECLARE(name, type, size) \
    struct \
    { \
        uint32_t      front;              /**< Front index. */ \
        uint32_t      back;               /**< Back index. */ \
        uint32_t      count;              /**< Current array size. */ \
        type          buffer[size];       /**< Array of items the deque contains. */ \
    } name

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Logs a buffer, provided its level. Also prints the file/line information.
    @brief    Macro to clear a deque.
    @param[in] deque        Pointer to the deque structure.
 **/
#define IFL_DEQUE_CLEAR(deque) \
    do \
    { \
        (deque)->front = 0; \
        (deque)->back = 0; \
        (deque)->count = 0; \
    } while (0)

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to initialize a deque.
    @param[in] deque        Pointer to the deque structure.
 **/
#define IFL_DEQUE_INIT(deque) \
    do \
    { \
        IFL_DEQUE_CLEAR(deque); \
    } while (0)

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to get maximum size of a deque.
    @param[in] deque        Pointer to the deque structure.
    @return                 The maximum number of items that the deque can contain.
 **/
#define IFL_DEQUE_MAX_SIZE(deque)     (IFL_ARRAY_SIZE((deque)->buffer))

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to get current size of a deque.
    @param[in] deque        Pointer to the deque structure.
    @return                 The current number of items currently hold in the deque.
 **/
#define IFL_DEQUE_SIZE(deque)        ((deque)->count)

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to check if a deque is full.
    @param[in] deque        Pointer to the deque structure.
    @retval                 true if there is no slot left in the deque.
    @retval                 false if there is at least one slot left in the deque.
 **/
#define IFL_DEQUE_FULL(deque)        (IFL_DEQUE_SIZE(deque) == IFL_DEQUE_MAX_SIZE(deque))

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to check if a deque is empty.
    @param[in] deque        Pointer to the deque structure.
    @retval                 true if there is no item in the deque.
    @retval                 false if there is at least one item in the deque.
 **/
#define IFL_DEQUE_EMPTY(deque)       (IFL_DEQUE_SIZE(deque) == 0)

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to get number of available slots in a deque.
    @param[in] deque        Pointer to the deque structure.
    @return                 The number of available slots in the deque.
 **/
#define IFL_DEQUE_AVAILABLE(deque)   (IFL_DEQUE_MAX_SIZE(deque) - IFL_DEQUE_SIZE(deque))

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro that returns the reference to the item at the back of the deque.
    @param[in] deque        Pointer to the deque structure.
    @param[out] data        Reference to the item.
    @warning                There is no checking done: must check the deque size before pushing
                            data.
 **/
#define IFL_DEQUE_BACK(deque, data) \
    do \
    { \
        uint32_t buffer_idx = (((deque)->back > 0) ? ((deque)->back - 1) : (uint32_t)(IFL_DEQUE_MAX_SIZE(deque) - 1)); \
        data = &(deque)->buffer[buffer_idx]; \
    } while (0)

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to push an item to the back of the deque.
    @param[in] deque        Pointer to the deque structure.
    @param[in] data         Pointer to the item to push.
    @warning                There is no checking done: must check the deque size before pushing the
                            data.
 **/
#define IFL_DEQUE_PUSH_BACK(deque, data) \
    do \
    { \
        (deque)->buffer[(deque)->back] = *data; \
        (deque)->back = (uint32_t) (((deque)->back == (IFL_DEQUE_MAX_SIZE(deque) - 1)) ? 0 : ((deque)->back + 1)); \
        (deque)->count++; \
    } while (0)

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to pop an item from the back of a deque (data is lost).
    @param[in] deque        Pointer to the deque structure.
    @warning                There is no checking done: must check the deque size before popping the
                            data.
 **/
#define IFL_DEQUE_POP_BACK(deque) \
    do \
    { \
        (deque)->back = (uint32_t)(((deque)->back > 0) ? ((deque)->back - 1) : (IFL_DEQUE_MAX_SIZE(deque) - 1)); \
        (deque)->count--; \
    } while (0)

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to return the reference to the item at the front of the deque.
    @param[in] deque        Pointer to the deque structure.
    @param[out] data        Reference to the deque structure.
    @warning                There is no checking done: must check the deque before getting the
                            data.
 **/

#define IFL_DEQUE_FRONT(deque, data) \
    do \
    { \
        data = &(deque)->buffer[(deque)->front]; \
    } while (0)

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to push an item to the front of the deque.
    @param[in] deque        Pointer to the deque structure.
    @param[in] data         Pointer to the item to push.
    @warning                There is no checking done: must check the deque size before pushing the
                            data.
 **/
#define IFL_DEQUE_PUSH_FRONT(deque, data) \
    do \
    { \
        (deque)->front = (uint32_t)(((deque)->front > 0) ? ((deque)->front - 1) : (IFL_DEQUE_MAX_SIZE(deque) - 1)); \
        (deque)->buffer[(deque)->front] = *data; \
        (deque)->count++; \
    } while (0)

/*  @ingroup  Firmware
    @ingroup  IFL
    @ingroup  HAL
    @ingroup  Deque
    @brief    Macro to pop an item from the front of a deque (data is lost).
    @param[in] deque        Pointer to the deque structure.
    @warning                There is no checking done: must check the deque size before popping the
                            data.[OK] Done.
 **/
#define IFL_DEQUE_POP_FRONT(deque) \
    do \
    { \
        (deque)->front = (uint32_t)(((deque)->front == (IFL_DEQUE_MAX_SIZE(deque) - 1)) ? 0 : ((deque)->front + 1)); \
        (deque)->count--; \
    } while (0)


// --------------------------------------------------------------------------------------------------------------------
// ----- public function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- public function prototypes
// --------------------------------------------------------------------------------------------------------------------


#endif // IFL_DEQUE_H
