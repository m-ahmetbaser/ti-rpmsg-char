/*
 * rpmsg_char_benchmark.c
 *
 * Benchmark Example application using rpmsg-char library
 *
 * Copyright (c) 2024-2025 Texas Instruments Incorporated - https://www.ti.com
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
#define MSG_LENGTH	100
#define MSG_LENGTH_MAX	496

/*
 * This test can plot round-trip latencies up to 9 sec in a histogram output.
 * A Linux system should not have 9 sec latencies. However, if there are any
 * latencies greater than 9 sec, they will be reported in the terminal, and then
 * the latencies will be rounded down to 9 sec for the
 * histogram plot and for worst-case and average latency calculations.
 *
 * Latencies are measured in microseconds (usec)
 */
#define LATENCY_RANGE 9000000

/*
 * This test was designed to run for no more than
 * 1,000,000,000,000 (1 Trillion) test runs.
 * More details in the "average latency" calculation below.
 * If more test runs are needed, the calculations for
 * average latency must be rewritten.
 */
#define NUM_MSGS_MAX 1000000000000

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
int rpmsg_char_ping(int rproc_id, char *dev_name, unsigned int local_endpt, unsigned int remote_endpt,
		long long num_msgs, int msg_length)
{
	int ret = 0;
	long long i = 0;
	int packet_len;
	char eptdev_name[64] = { 0 };
	/*
	 * Each RPMsg packet can have up to 496 bytes of data:
	 * 512 bytes total - 16 byte header = 496
	 */
	char packet_buf[496] = { 0 };
	char rx_packet_buf[496] = { 0 };
	rpmsg_char_dev_t *rcdev;
	int flags = 0;
	struct timespec ts_current;
	struct timespec ts_end;

	/*
	 * Variables used for latency benchmarks. Any variable that scales with
	 * the number of test runs is set to 64 bits minimum with long long int.
	 */
	struct timespec ts_start_test;
	struct timespec ts_end_test;
	/* latency measured in us */
	int latency = 0;
	long long latencies[LATENCY_RANGE + 1] = {0};
	int latency_worst_case = 0;
	long long latency_average = 0;
	FILE *file_ptr;

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

	printf("Exchanging %lld messages with rpmsg device %s on rproc id %d ...\n\n",
		num_msgs, eptdev_name, rproc_id);

	clock_gettime(CLOCK_MONOTONIC, &ts_start_test);

	memset(packet_buf, 0, sizeof(packet_buf));

	/* select how many bytes to send per message */

	/* send 1 byte */
	//sprintf(packet_buf, "0");
	/* send 4 bytes */
	//sprintf(packet_buf, "0123");
	/* send 32 bytes */
	//sprintf(packet_buf, "01234567890123456789012345678901");
	/* "normal" test: do the hello message */
	//sprintf(packet_buf, "hello there %d!", i);
	/* maximum test: send 496 bytes (i.e., 495 bytes plus null termination) */
	//sprintf(packet_buf, "012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234");

	/*
	 * Does the number of 1s getting written matter?
	 * test with '@' (0100_0000)
	 * and
	 * 'w' (0111_0111)
	 */
	for (i = 0; i < msg_length; i++) {
		packet_buf[i] = '@';
		// packet_buf[i] = 'w';
	}

	/*
	 * packet_len scales with the size of the message to send.
	 * This allows us to spend fewer clock cycles reading & writing
	 * if we send and receive less data.
	 */
	packet_len = strlen(packet_buf);
	if (packet_len != msg_length) {
		printf("Error: packet_len != msg_length \n");
		printf("packet_len = %d, msg_length = %d\n", packet_len, msg_length);
	}

	for (i = 0; i < num_msgs; i++) {

		/* remove prints to speed up the test execution time */
		//printf("Sending message #%d: %s\n", i, packet_buf);

		clock_gettime(CLOCK_MONOTONIC, &ts_current);
		ret = send_msg(rcdev->fd, (char *)packet_buf, packet_len);
		if (ret < 0) {
			printf("send_msg failed for iteration %lld, ret = %d\n", i, ret);
			goto out;
		}
		if (ret != packet_len) {
			printf("bytes written does not match send request, ret = %lld, packet_len = %d\n",
				i, ret);
			goto out;
		}

		ret = recv_msg(rcdev->fd, packet_len, (char *)rx_packet_buf, &packet_len);
		clock_gettime(CLOCK_MONOTONIC, &ts_end);
		if (ret < 0) {
			printf("recv_msg failed for iteration %lld, ret = %d\n", i, ret);
			goto out;
		}

		/* remove prints to speed up the test execution time */
		//printf("Received message #%d: round trip delay(usecs) = %ld\n", i,(ts_end.tv_nsec - ts_current.tv_nsec)/1000);
		//printf("%s\n", rx_packet_buf);

		/* latency measured in usec */
		latency = (ts_end.tv_nsec - ts_current.tv_nsec) / 1000;

		/*
		 * If latency > latency_worst_case:
		 * - If latency > LATENCY_RANGE:
		 *     - Print latency
		 *     - Set latency = LATENCY_RANGE
		 *       This ensures we get notified of every latency > LATENCY_RANGE
		 * - set latency_worst_case = latency
		 * - Continue test
		 */
		if (latency > latency_worst_case) {
			if (latency > LATENCY_RANGE) {
				printf("Large latency measured: %d usec\n", latency);
				printf("latency will be rounded to 9,000,000 usec for histogram & calculations.\n");
				latency = LATENCY_RANGE;
			}
			latency_worst_case = latency;
		}

		/* increment the counter for that specific latency measurement */
		latencies[latency]++;
	}

	clock_gettime(CLOCK_MONOTONIC, &ts_end_test);

	/*
	 * Find the average latency
	 * max value of a long long int: 9,223,372,036,854,775,807
	 * Let's keep any variable from going above 9,000,000,000,000,000,000.
	 * Let's set 1 Trillion as the max test iterations (1,000,000,000,000).
	 * Thus, the max latency that can be recorded is
	 * 9,000,000,000,000,000,000 / 1,000,000,000,000 = 9,000,000 usec = 9sec
	 *
	 * Realistically, a Linux system should never have a 9 sec delay. But
	 * keep in mind that any latencies above 9 sec will get truncated to
	 * 9 sec.
	 */

	/*
	 * Iterate through all possible latency measurements between
	 * latency_worst_case and 0 usec
	 */
	for (i = latency_worst_case; i > 0; i--) {
		/* get the weighted average of each latency measurement */
		/* e.g., if latencies[60] = 17, that means there was a latency of 60us 17 times */
		latency_average = latency_average + (latencies[i] * i);
	}
	latency_average = latency_average / num_msgs;

	/*
	 * Export the latency measurements to a file that can be used to
	 * generate a histogram plot.
	 */
	file_ptr = fopen("histogram.txt", "w");

	for (i = 0; i <= latency_worst_case; i++)
	{
		fprintf(file_ptr, "%lld , ", i);
		fprintf(file_ptr, "%lld", latencies[i]);
		fprintf(file_ptr, "\n");
	}
	fclose(file_ptr);

	printf("\nCommunicated %lld messages successfully on %s\n\n",
		num_msgs, eptdev_name);
	printf("Total execution time for the test: %ld seconds\n",
		ts_end_test.tv_sec - ts_start_test.tv_sec);
	printf("Average round-trip latency: %lld\n", latency_average);
	printf("Worst-case round-trip latency: %d\n", latency_worst_case);
	printf("Histogram data at histogram.txt\n");

out:
	ret = rpmsg_char_close(rcdev);
	if (ret < 0)
		perror("Can't delete the endpoint device\n");

	return ret;
}

void usage()
{
	printf("Usage: rpmsg_char_benchmark [-r <rproc_id>] [-n <num_msgs>] \
	        [-m <msg_length>] [-d \
	        <rpmsg_dev_name>] [-p <remote_endpt>] [-l <local_endpt>] \n");
	printf("\t\tDefaults: rproc_id: 0, num_msgs: %d, msg_length: %d, \
	        rpmsg_dev_name: NULL remote_endpt: %d\n",
		NUM_ITERATIONS, MSG_LENGTH, REMOTE_ENDPT);
}

int main(int argc, char *argv[])
{
	int ret, status, c, msg_length;
	int rproc_id = 0;
	long long num_msgs = NUM_ITERATIONS;
	unsigned int remote_endpt = REMOTE_ENDPT;
	unsigned int local_endpt = RPMSG_ADDR_ANY;
	char *dev_name = NULL;

	while (1) {
		c = getopt(argc, argv, "r:n:m:p:d:l:");
		if (c == -1)
			break;

		switch (c) {
		case 'r':
			rproc_id = atoi(optarg);
			break;
		case 'n':
			num_msgs = atoi(optarg);
			break;
		case 'm':
			msg_length = atoi(optarg);
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

	if (num_msgs > NUM_MSGS_MAX) {
		printf("Invalid number of messages %lld, should be less than %ld\n",
			num_msgs, NUM_MSGS_MAX);
		usage();
		return 1;
	}

	if (msg_length > MSG_LENGTH_MAX) {
		printf("Invalid message length %d, should be less than or \
		        equal to %d\n", msg_length, MSG_LENGTH_MAX);
		usage();
		return 1;
	}

	/* Use auto-detection for SoC */
	ret = rpmsg_char_init(NULL);
	if (ret) {
		printf("rpmsg_char_init failed, ret = %d\n", ret);
		return ret;
	}

	status = rpmsg_char_ping(rproc_id, dev_name, local_endpt, remote_endpt, num_msgs, msg_length);

	rpmsg_char_exit();

	if (status < 0) {
		printf("TEST STATUS: FAILED\n");
	} else {
		printf("TEST STATUS: PASSED\n");
	}

	return 0;
}
