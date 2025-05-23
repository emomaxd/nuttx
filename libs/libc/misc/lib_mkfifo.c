/****************************************************************************
 * libs/libc/misc/lib_mkfifo.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

#if defined(CONFIG_PIPES) && CONFIG_DEV_FIFO_SIZE > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkfifo
 *
 * Description:
 *   mkfifo() makes a FIFO device driver file with name 'pathname.'  Unlike
 *   Linux, a NuttX FIFO is not a special file type but simply a device
 *   driver instance.  'mode' specifies the FIFO's permissions.
 *
 *   Once the FIFO has been created by mkfifo(), any thread can open it for
 *   reading or writing, in the same way as an ordinary file. However, it
 *   must have been opened from both reading and writing before input or
 *   output can be performed.  This FIFO implementation will block all
 *   attempts to open a FIFO read-only until at least one thread has opened
 *   the FIFO for  writing.
 *
 *   If all threads that write to the FIFO have closed, subsequent calls to
 *   read() on the FIFO will return 0 (end-of-file).
 *
 * Input Parameters:
 *   pathname - The full path to the FIFO instance to attach to or to create
 *     (if not already created).
 *   mode - Ignored for now
 *
 * Returned Value:
 *   0 is returned on success; otherwise, -1 is returned with errno set
 *   appropriately.
 *
 ****************************************************************************/

int mkfifo(FAR const char *pathname, mode_t mode)
{
  int ret;

  ret = nx_mkfifo(pathname, mode, CONFIG_DEV_FIFO_SIZE);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: mkfifoat
 *
 * Description:
 *   The mkfifoat() system call operates in exactly the same way as mkfifo(),
 *   except for  the  differences described here.
 *
 *   If the pathname given in pathname is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor dirfd
 *   (rather than relative to the current working directory of the calling
 *    process)
 *
 *   If pathname is relative and dirfd is the special value AT_FDCWD, then
 *   pathname is interpreted relative to the current working directory of
 *   the calling process (like mkfifo()).
 *
 *   If pathname is absolute, then dirfd is ignored.
 *
 * Input Parameters:
 *   dirfd - The file descriptor of directory.
 *   path  - a pointer to the path.
 *   mode  - the access mode.
 *
 * Returned Value:
 *   Return zero on success, or -1 if an error occurred (in which case,
 *   errno is set appropriately).
 *
 ****************************************************************************/

int mkfifoat(int dirfd, FAR const char *path, mode_t mode)
{
  FAR char *fullpath;
  int ret;

  fullpath = lib_get_pathbuffer();
  if (fullpath == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  ret = lib_getfullpath(dirfd, path, fullpath, PATH_MAX);
  if (ret < 0)
    {
      lib_put_pathbuffer(fullpath);
      set_errno(-ret);
      return ERROR;
    }

  ret = mkfifo(fullpath, mode);
  lib_put_pathbuffer(fullpath);
  return ret;
}

#endif /* CONFIG_PIPES && CONFIG_DEV_FIFO_SIZE > 0 */
