/*
 * rpmsg_char_simple.c
 *
 * Simple Example application using rpmsg-char library
 *
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

#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stddef.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <stdbool.h>
#include <semaphore.h>

#include <linux/rpmsg.h>
#include <ti_rpmsg_char.h>

#define NUM_ITERATIONS	100
#define REMOTE_ENDPT	14

int send_msg(int fd, char *msg, int len)
{
	int ret = 0;

	ret = write(fd, msg, len);
	if (ret < 0) {
		perror("Can't write to rpmsg endpt device\n");
		return -1;
	}

	return ret;
}

int recv_msg(int fd, int len, char *reply_msg, int *reply_len)
{
	int ret = 0;

	/* Note: len should be max length of response expected */
	ret = read(fd, reply_msg, len);
	if (ret < 0) {
		perror("Can't read from rpmsg endpt device\n");
		return -1;
	} else {
		*reply_len = ret;
	}

	return 0;
}

/* single thread communicating with a single endpoint */
int rpmsg_char_ping(int rproc_id, char *dev_name, int local_endpt, int remote_endpt,
		    int num_msgs)
{
	int ret = 0;
	int i = 0;
	int packet_len;
	char eptdev_name[32] = { 0 };
	char packet_buf[512] = { 0 };
	rpmsg_char_dev_t *rcdev;
	int flags = 0;

        /*
         * Open the remote rpmsg device identified by dev_name and bind the
	 * device to a local end-point used for receiving messages from
	 * remote processor
         */
	sprintf(eptdev_name, "rpmsg-char-%d-%d", rproc_id, getpid());
	rcdev = rpmsg_char_open(rproc_id, dev_name, local_endpt, remote_endpt,
				eptdev_name, flags);
        if (!rcdev) {
		perror("Can't create an endpoint device");
		return -EPERM;
        }
        printf("Created endpt device %s, fd = %d port = %d\n", eptdev_name,
		rcdev->fd, rcdev->endpt);

        printf("Exchanging %d messages with rpmsg device ti.ipc4.ping-pong on rproc id %d ...\n\n",
		num_msgs, rproc_id);

	for (i = 0; i < num_msgs; i++) {
		memset(packet_buf, 0, sizeof(packet_buf));
		sprintf(packet_buf, "hello there %d!", i);
		packet_len = strlen(packet_buf);
		printf("Sending message #%d: %s\n", i, packet_buf);
		ret = send_msg(rcdev->fd, (char *)packet_buf, packet_len);
		if (ret < 0) {
			printf("send_msg failed for iteration %d, ret = %d\n", i, ret);
			goto out;
		}
		if (ret != packet_len) {
			printf("bytes written does not match send request, ret = %d, packet_len = %d\n",
				i, ret);
		    goto out;
		}

		printf("Receiving message #%d: ", i);
		ret = recv_msg(rcdev->fd, 256, (char *)packet_buf, &packet_len);
		if (ret < 0) {
			printf("recv_msg failed for iteration %d, ret = %d\n", i, ret);
			goto out;
		}
		/* TODO: Verify data integrity */

		/* TODO: Reduce number of prints */
		printf("%s\n", packet_buf);
	}

	printf("\nCommunicated %d messages successfully on %s\n\n",
		num_msgs, eptdev_name);

out:
	ret = rpmsg_char_close(rcdev);
	if (ret < 0)
		perror("Can't delete the endpoint device\n");

	return ret;
}

void usage()
{
	printf("Usage: rpmsg_char_simple [-r <rproc_id>] [-n <num_msgs>] [-d \
	       <rpmsg_dev_name] [-p <remote_endpt] [-l <local_endpt] \n");
	printf("\t\tDefaults: rproc_id: 0 num_msgs: %d rpmsg_dev_name: NULL remote_endpt: %d\n",
		NUM_ITERATIONS, REMOTE_ENDPT);
}

int main(int argc, char *argv[])
{
	int ret, status, c;
	int rproc_id = 0;
	int num_msgs = NUM_ITERATIONS;
	int remote_endpt = REMOTE_ENDPT;
	int local_endpt = RPMSG_ADDR_ANY;
	char *dev_name = NULL;

	while (1) {
		c = getopt(argc, argv, "r:n:p:d:l:");
		if (c == -1)
			break;

		switch (c) {
		case 'r':
			rproc_id = atoi(optarg);
			break;
		case 'n':
			num_msgs = atoi(optarg);
			break;
		case 'p':
			remote_endpt = atoi(optarg);
			break;
		case 'd':
			dev_name = optarg;
			break;
		case 'l':
			local_endpt = atoi(optarg);
			break;
		default:
			usage();
			exit(0);
		}
	}

	if (rproc_id < 0 || rproc_id >= RPROC_ID_MAX) {
		printf("Invalid rproc id %d, should be less than %d\n",
			rproc_id, RPROC_ID_MAX);
		usage();
		return 1;
	}

	/* Use auto-detection for SoC */
	ret = rpmsg_char_init(NULL);
	if (ret) {
		printf("rpmsg_char_init failed, ret = %d\n", ret);
		return ret;
	}

	status = rpmsg_char_ping(rproc_id, dev_name, local_endpt, remote_endpt, num_msgs);

	rpmsg_char_exit();

	if (status < 0) {
		printf("TEST STATUS: FAILED\n");
	} else {
		printf("TEST STATUS: PASSED\n");
	}

	return 0;
}
