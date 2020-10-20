/*
 * Copyright (c) 2020 Texas Instruments Incorporated - https://www.ti.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/rpmsg.h>

#define RPMSG_ADDR_ANY		0xFFFFFFFF

#include "ti_rpmsg_char.h"
#include "rpmsg_char_internal.h"

#define printerr printf

#define to_rpmsg_char_endpt(ptr) ({				\
		const rpmsg_char_dev_t *_mptr = (ptr);		\
		(struct rpmsg_char_endpt *)((char *)_mptr -	\
			offsetof(struct rpmsg_char_endpt, rcdev)); })

struct rpmsg_char_endpt {
	struct rpmsg_char_endpt *next;
	const struct rproc_map *map;
	char *rpath;
	struct rpmsg_char_dev rcdev;
	char rpmsg_dev_name[64];
	int remoteproc_id;
	int virtio_id;
	int ctrl_id;
	int rpmsg_id;
};

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

static bool inited = false;
static int soc_id = -1;
static struct soc_rprocs soc_data;
static struct rpmsg_char_endpt *ghead = NULL;
static struct rpmsg_char_endpt *gtail = NULL;

static void _list_add(struct rpmsg_char_endpt *ept)
{
	if (!ghead)
		ghead = ept;

	if (gtail)
		gtail->next = ept;
	gtail = ept;
}

static void _list_remove(struct rpmsg_char_endpt *ept)
{
	struct rpmsg_char_endpt *iter, *prev;

	if (ghead == ept) {
		if (gtail == ept)
			gtail = NULL;
		ghead = ept->next;
		return;
	}

	prev = ghead;
	for (iter = ghead->next; iter != NULL; prev = iter, iter = iter->next) {
		if (iter == ept) {
			if (prev)
				prev->next = ept->next;
			if (gtail == ept)
				gtail = prev;
			return;
		}
	}
}

static bool _list_is_present(struct rpmsg_char_endpt *ept)
{
	struct rpmsg_char_endpt *iter;

	for (iter = ghead; iter != NULL; iter = iter->next) {
		if (iter == ept)
			return true;
	}

	return false;
}

static int _rpmsg_char_check_system(void)
{
	if (check_dir("/sys/class/remoteproc")) {
		fprintf(stderr, "remoteproc framework is not enabled/installed\n");
		return -ENOENT;
	}

	if (check_dir("/sys/bus/virtio")) {
		fprintf(stderr, "virtio framework is not enabled/installed\n");
		return -ENOENT;
	}

	if (check_dir("/sys/bus/rpmsg")) {
		fprintf(stderr, "rpmsg bus driver is not enabled/installed\n");
		return -ENOENT;
	}

	if (check_dir("/sys/class/rpmsg")) {
		fprintf(stderr, "rpmsg_chrdev driver is not enabled/installed\n");
		return -ENOENT;
	}

	return 0;
}

/* parse sysfs to find associated kernel-allocated remoteproc and virtio ids */
static int _rpmsg_char_find_rproc(struct rpmsg_char_endpt *ept,
				  enum rproc_id id)
{
	const struct rproc_map *r = soc_data.map;
	unsigned int remoteproc_id;
	unsigned int virtio_id;
	char fpath[512];
	int i;
	int ret = -ENODEV;

	for (i = 0; i < soc_data.num_rprocs; i++, r++) {
		if (id == r->id)
			break;
	}
	if (i == soc_data.num_rprocs) {
		fprintf(stderr, "%s: SoC doesn't have rproc id %d\n",
			__func__, id);
		return -EINVAL;
	}

	ept->rpath = file_deref_link("/sys/bus/platform/devices", r->rproc_name);
	if (check_dir(ept->rpath)) {
		fprintf(stderr, "%s: %s device is mostly yet to be created!\n",
			__func__, r->rproc_name);
		return -ENOENT;
	}

	/* retrieve the dynamically created kernel remoteproc id */
	memset(&fpath, 0, sizeof(fpath));
	sprintf(fpath, "%s/remoteproc", ept->rpath);
	ret = get_child_dir_suffix(fpath, "remoteproc%u", &remoteproc_id);
	if (ret) {
		if (ret == -ENOENT) {
			fprintf(stderr, "%s: %s is either not probed or not a remoteproc!\n",
				__func__, r->rproc_name);
		}
		return ret;
	}

	/* retrieve the dynamically created kernel virtio id */
	memset(&fpath, 0, sizeof(fpath));
	sprintf(fpath, "%s/remoteproc/remoteproc%u/remoteproc%u#vdev0buffer",
		ept->rpath, remoteproc_id, remoteproc_id);
	ret = get_child_dir_suffix(fpath, "virtio%u", &virtio_id);
	if (ret) {
		if (ret == -ENOENT) {
			fprintf(stderr, "%s: %s does not have any virtio devices!\n",
				__func__, r->rproc_name);
		}
		return ret;
	}

	ept->map = r;
	ept->remoteproc_id = remoteproc_id;
	ept->virtio_id = virtio_id;

	return 0;
}

static int _rpmsg_char_find_ctrldev(struct rpmsg_char_endpt *ept,
				    char *dev_name, int remote_port)
{
	char virtio[16] = { 0 };
	struct dirent *iter;
	DIR *dir;
	char fpath[512];
	char *rpath;
	unsigned int ctrl_id;
	int ret;

	sprintf(virtio, "virtio%u", ept->virtio_id);
	rpath = file_deref_link("/sys/bus/virtio/devices", virtio);
	if (!rpath)
		return -ENOENT;

	sprintf(ept->rpmsg_dev_name, "virtio%u.%s.-1.%d", ept->virtio_id,
		dev_name, remote_port);

	dir = opendir(rpath);
	if (!dir) {
		ret = -errno;
		goto free_rpath;
	}

	/*
	 * find a directory that matches the published rpmsg device of the
	 * form virtio%d.<rpmsg_dev.%d.%d
	 */
	while ((iter = readdir(dir))) {
		if (iter->d_type == DT_DIR) {
			if (!strcmp(iter->d_name, ept->rpmsg_dev_name))
				break;
		}
	}
	if (!iter) {
		fprintf(stderr, "%s: could not find the matching rpmsg_ctrl device for %s\n",
			__func__, ept->rpmsg_dev_name);
		ret = -ENOENT;
		goto chrdev_nodir;
	}

	memset(&fpath, 0, sizeof(fpath));
	sprintf(fpath, "%s/%s/rpmsg", rpath, iter->d_name);
	if (check_dir(fpath)) {
		fprintf(stderr, "%s: rpmsg directory doesn't exist under %s\n",
			__func__, ept->rpmsg_dev_name);
		ret = -ENOENT;
		goto chrdev_nodir;
	}

	/* get rpmsg_ctrl id */
	ret = get_child_dir_suffix(fpath, "rpmsg_ctrl%u", &ctrl_id);
	if (ret)
		goto chrdev_nodir;

	ept->ctrl_id = ctrl_id;

chrdev_nodir:
	closedir(dir);
free_rpath:
	free(rpath);
	return ret;
}

static int _rpmsg_char_create_eptdev(struct rpmsg_char_endpt *ept,
				     char *eptdev_name, int remote_port)
{
	int fd, ret;
	char ctrldev_path[32] = { 0 };
	struct rpmsg_endpoint_info ept_info = { 0 };

	sprintf(ctrldev_path, "/dev/rpmsg_ctrl%u", ept->ctrl_id);

	fd = open(ctrldev_path, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "%s: could not open rpmsg_ctrl dev node for id %d\n",
			__func__, ept->ctrl_id);
		return fd;
	}

	/* let kernel dynamically allocate the local end-point */
	ept_info.src = RPMSG_ADDR_ANY;
	ept_info.dst = remote_port;
	sprintf(ept_info.name, "%s", eptdev_name);

	ret = ioctl(fd, RPMSG_CREATE_EPT_IOCTL, &ept_info);
	if (ret) {
		fprintf(stderr, "%s: ctrl_fd ioctl: %s\n", __func__,
			strerror(errno));
	}

	close(fd);

	return ret;
}

static int _rpmsg_char_destroy_eptdev(int fd)
{
	int ret;

	ret = ioctl(fd, RPMSG_DESTROY_EPT_IOCTL, NULL);
	if (ret) {
		fprintf(stderr, "%s: could not destroy endpt %d\n",
			__func__, fd);
		return ret;
	}

	close(fd);
	return 0;
}

static int _rpmsg_char_get_rpmsg_id(struct rpmsg_char_endpt *ept,
				    char *eptdev_name)
{
	const struct rproc_map *r = ept->map;
	bool found = false;
	char rpmsg[16] = { 0 };
	char fpath[512];
	char dev_name[32];
	char *rpath;
	struct dirent *iter;
	DIR *dir;
	int ret;

	sprintf(rpmsg, "rpmsg_ctrl%u", ept->ctrl_id);
	rpath = file_deref_link("/sys/class/rpmsg", rpmsg);
	if (!rpath)
		return -ENOENT;

	dir = opendir(rpath);
	if (!dir) {
		free(rpath);
		return -errno;
	}

	while ((iter = readdir(dir))) {
		if (iter->d_type != DT_DIR ||
		    strncmp(iter->d_name, "rpmsg", strlen("rpmsg")))
			continue;

		memset(&fpath, 0, sizeof(fpath));
		memset(&dev_name, 0, sizeof(dev_name));
		sprintf(fpath, "%s/%s/name", rpath, iter->d_name);
		ret = file_read_string(fpath, dev_name, sizeof(dev_name));
		if (ret < 0)
			continue;

		if (strcmp(dev_name, eptdev_name))
			continue;

		sscanf(iter->d_name, "rpmsg%u", &ept->rpmsg_id);
		found = true;
		break;
	}

	free(rpath);
	closedir(dir);
	return found ? 0 : -1;
}

static int _rpmsg_char_get_local_endpt(struct rpmsg_char_endpt *ept)
{
	const struct rproc_map *r = ept->map;
	char fpath[512] = { 0 };
	char rpmsg[16] = { 0 };
	char *rpath;

	sprintf(rpmsg, "rpmsg%u", ept->rpmsg_id);
	rpath = file_deref_link("/sys/class/rpmsg", rpmsg);
	if (!rpath) {
		fprintf(stderr, "%s: rpmsg%u realpath failed\n",
			__func__, ept->rpmsg_id);
		return -ENOENT;
	}

	sprintf(fpath, "%s/src", rpath);
	free(rpath);

	return file_read_value(fpath);
}

static int _rpmsg_char_open_eptdev(struct rpmsg_char_endpt *ept,
				   char *eptdev_name, int flags)
{
	char eptdev_path[32] = { 0 };
	int efd, endpt;
	int ret;

	ret = _rpmsg_char_get_rpmsg_id(ept, eptdev_name);
	if (ret < 0)
		return ret;

	sprintf(eptdev_path, "/dev/rpmsg%u", ept->rpmsg_id);

	efd = open(eptdev_path, O_RDWR | flags);
	if (efd < 0) {
		fprintf(stderr, "failed to open eptdev %s\n", eptdev_path);
		return -errno;
	}

	/* local end-point is assigned after the open call */
	endpt = _rpmsg_char_get_local_endpt(ept);
	if (endpt < 0) {
		_rpmsg_char_destroy_eptdev(efd);
		return endpt;
	}

	ept->rcdev.fd = efd;
	ept->rcdev.endpt = endpt;

	return 0;
}

static struct rpmsg_char_endpt *_rpmsg_char_endpt_alloc(void)
{
	struct rpmsg_char_endpt *ept;

	ept = calloc(sizeof(*ept), 1);
	if (!ept) {
		fprintf(stderr, "%s: failed to allocate memory\n", __func__);
		return NULL;
	}

	ept->remoteproc_id = -1;
	ept->virtio_id = -1;
	ept->ctrl_id = -1;
	ept->rpmsg_id = -1;

	return ept;
}

static void _rpmsg_char_endpt_free(struct rpmsg_char_endpt *ept)
{
	if (ept->rpath)
		free(ept->rpath);
	free(ept);
}

static void _rpmsg_char_cleanup(void)
{
	struct rpmsg_char_endpt *iter;
	int ret;

	if (ghead)
		fprintf(stderr, "Application did not close some rpmsg_char devices\n");

	for (iter = ghead; iter != NULL; iter = iter->next) {
		ret = rpmsg_char_close(&iter->rcdev);
		if (ret) {
			fprintf(stderr, "rpmsg_char_close failed during cleanup, rcdev = 0x%x, ret = %d\n",
				&iter->rcdev, ret);
		}
	}
}

static void signal_handler(int sig)
{
	fprintf(stderr, "\nClean up and exit while handling signal %d\n", sig);
	rpmsg_char_exit();
	exit(EXIT_FAILURE);
}

rpmsg_char_dev_t *rpmsg_char_open(enum rproc_id id, char *dev_name,
				  int remote_endpt, char *eptdev_name,
				  int flags)
{
	struct rpmsg_char_endpt *ept = NULL;
	char *def_dev_name = "rpmsg_chrdev";
	int ret, local_endpt;

	pthread_mutex_lock(&mutex);

	if (!inited) {
		fprintf(stderr, "%s cannot be invoked without initialization\n",
			__func__);
		goto unlock;
	}

	if (remote_endpt == RPMSG_ADDR_ANY) {
		fprintf(stderr, "%s: remote_endpt cannot be RPMSG_ADDR_ANY\n",
			__func__);
		goto unlock;
	}

	if (!eptdev_name || strlen(eptdev_name) >= 32) {
		fprintf(stderr, "%s: invalid eptdev_name\n", __func__);
		goto unlock;
	}

	if (!dev_name)
		dev_name = def_dev_name;

	ept = _rpmsg_char_endpt_alloc();
	if (!ept)
		goto unlock;

	ret = _rpmsg_char_find_rproc(ept, id);
	if (ret)
		goto out;

	ret = _rpmsg_char_find_ctrldev(ept, dev_name, remote_endpt);
	if (ret)
		goto out;

	ret = _rpmsg_char_create_eptdev(ept, eptdev_name, remote_endpt);
	if (ret)
		goto out;

	ret = _rpmsg_char_open_eptdev(ept, eptdev_name, flags);
	if (ret < 0)
		goto out;

	_list_add(ept);
	goto unlock;

out:
	_rpmsg_char_endpt_free(ept);
	ept = NULL;
unlock:
	pthread_mutex_unlock(&mutex);
	return ept ? &ept->rcdev : NULL;
}

int rpmsg_char_close(rpmsg_char_dev_t *rcdev)
{
	int ret;
	struct rpmsg_char_endpt *ept;

	if (!rcdev)
		return -EINVAL;

	pthread_mutex_lock(&mutex);

	ept = to_rpmsg_char_endpt(rcdev);
	if (!_list_is_present(ept)) {
		fprintf(stderr, "%s: invalid handle passed in rcdev = 0x%x\n",
			__func__, rcdev);
		ret = -ENOENT;
		goto out;
	}

	ret = _rpmsg_char_destroy_eptdev(rcdev->fd);
	if (ret)
		goto out;

	_list_remove(ept);
	_rpmsg_char_endpt_free(ept);

out:
	pthread_mutex_unlock(&mutex);
	return ret;
}

int rpmsg_char_init(char *soc_name)
{
	int ret = 0;

	pthread_mutex_lock(&mutex);

	if (inited) {
		ret = 1;
		goto out;
	}

	ret = _rpmsg_char_check_system();
	if (ret < 0)
		goto out;

	ret = _rpmsg_char_find_soc_family(soc_name, &soc_data);
	if (ret < 0)
		goto out;

	/*
	 * Setup the signal handlers, this ideally needs to be done in
	 * applications
	 */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	inited = true;

out:
	pthread_mutex_unlock(&mutex);
	return ret;
}

void rpmsg_char_exit(void)
{
	pthread_mutex_lock(&mutex);
	if (!inited) {
		fprintf(stderr, "%s: library not initialized previously\n",
			__func__);
		pthread_mutex_unlock(&mutex);
		return;
	}
	pthread_mutex_unlock(&mutex);

	_rpmsg_char_cleanup();

	pthread_mutex_lock(&mutex);
	inited = false;
	pthread_mutex_unlock(&mutex);
}
