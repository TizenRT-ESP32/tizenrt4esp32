/****************************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
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
//===----------------------------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

// <forward_list>

// template <class Compare> static int merge(forward_list&& x, Compare comp);

#include <forward_list>
#include <iterator>
#include <functional>
#include <cassert>
#include "libcxx_tc_common.h"


int tc_libcxx_containers_forwardlist_ops_merge_pred(void)
{
    {
        typedef int T;
        typedef std::forward_list<T> C;
        const T t1[] = {13, 12, 7, 6, 5, 3};
        const T t2[] = {15, 14, 11, 10, 9, 8, 4, 2, 1, 0};
        const T t3[] = {15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
        C c1(std::begin(t1), std::end(t1));
        C c2(std::begin(t2), std::end(t2));
        c1.merge(c2, std::greater<T>());
        C c3(std::begin(t3), std::end(t3));
        TC_ASSERT_EXPR(c1 == c3);
    }
    TC_SUCCESS_RESULT();
    return 0;
}
