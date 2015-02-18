#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <inttypes.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "sysfs.h"

int parse_sysfs_string(char *filename, char *string, size_t len)
{
	FILE *f;
	char buf[1024];
	char *r;

	f = fopen(filename, "r");
	if (f == NULL)
		return -1;

	r = fgets(buf, sizeof(buf), f);
	if (r == NULL) {
		fclose(f);
		return -1;
	}

	/* remove the '\n' at the end */
	buf[strlen(buf) - 1] = '\0';
	snprintf(string, len, "%s", buf);

	fclose(f);

	return 1;
}

int parse_sysfs_int(char *filename, uint64_t *value)
{
	FILE *f;
	char buf[1024];
	char *r;
	char *endptr;

	f = fopen(filename, "r");
	if (f == NULL)
		return -1;

	r = fgets(buf, sizeof(buf), f);
	if (r == NULL) {
		fclose(f);
		return -1;
	}

	errno = 0;
	endptr = NULL;
	*value = strtoul(buf, &endptr, 0);
	if (errno != 0 || endptr == NULL || *endptr != '\n') {
		fprintf(stderr, "strtoul failed errno: %d\n", errno);
		fclose(f);
		return -1;
	}

	fclose(f);

	return 1;
}
