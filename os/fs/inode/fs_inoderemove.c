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
 * fs/inode/fs_inoderemove.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <errno.h>

#include <tinyara/kmalloc.h>
#include <tinyara/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_unlink
 *
 * Description:
 *   Given a path, remove a the node from the in-memory, inode tree that the
 *   path refers to.  This is normally done in preparation to removing or
 *   moving an inode.
 *
 * Assumptions/Limitations:
 *   The caller must hold the inode semaphore
 *
 ****************************************************************************/

FAR struct inode *inode_unlink(FAR const char *path)
{
	const char *name = path;
	FAR struct inode *node;
	FAR struct inode *peer;
	FAR struct inode *parent;

	/* Verify parameters.  Ignore null paths and relative paths */

	if (!path || *path == '\0' || path[0] != '/') {
		return NULL;
	}

	/* Find the node to unlink */

	node = inode_search(&name, &peer, &parent, (const char **)NULL);
	if (node) {
		/* If peer is non-null, then remove the node from the right of
		 * of that peer node.
		 */

		if (peer) {
			peer->i_peer = node->i_peer;
		}

		/* If parent is non-null, then remove the node from head of
		 * of the list of children.
		 */

		else if (parent) {
			parent->i_child = node->i_peer;
		}

		/* Otherwise, we must be removing the root inode. */

		else {
			root_inode = node->i_peer;
		}

		node->i_peer = NULL;
	}

	return node;
}

/****************************************************************************
 * Name: inode_remove
 *
 * Description:
 *   Given a path, remove a the node from the in-memory, inode tree that the
 *   path refers to and free all resources related to the inode.  If the
 *   inode is in-use, then it will be unlinked, but will not be freed until
 *   the last reference to the inode is released.
 *
 * Assumptions/Limitations:
 *   The caller must hold the inode semaphore
 *
 ****************************************************************************/

int inode_remove(FAR const char *path)
{
	FAR struct inode *node;

	/* Find the inode and unlink it from the in-memory inode tree */

	node = inode_unlink(path);
	if (node) {
		/* Found it! But we cannot delete the inode if there are references
		 * to it
		 */

		if (node->i_crefs) {
			/* In that case, we will mark it deleted, when the filesystem
			 * releases the inode, we will then, finally delete the subtree
			 */

			node->i_flags |= FSNODEFLAG_DELETED;
			return -EBUSY;
		} else {
			/* And delete it now -- recursively to delete all of its children */

			inode_free(node->i_child);
			kmm_free(node);
			return OK;
		}
	}

	/* The node does not exist */

	return -ENOENT;
}
