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

#define SYSFS_PCI_PATH "/sys/bus/pci/devices/"

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

int find_sysfs_device(uint64_t vendor_id, uint64_t device_id,
		char *directory, size_t directory_len)
{
	int r;
	DIR *dir;
	struct dirent *e;

	if (directory == NULL || directory_len <= 0)
		return -1;

	dir = opendir(SYSFS_PCI_PATH);
	if (dir == NULL)
		return -1;

	while ((e = readdir(dir)) != NULL) {
		char path[PATH_MAX];
		char device_file[PATH_MAX];
		char vendor_file[PATH_MAX];
		uint64_t vid;
		uint64_t did;
		struct stat stbuf;

		if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0)
			continue;

		strncpy(path, SYSFS_PCI_PATH, PATH_MAX);
		strncat(path, e->d_name, PATH_MAX - strlen(path));

		r = stat(path, &stbuf);
		if (r < 0) {
			fprintf(stderr, "stat failed: %s\n", strerror(errno));
			continue;
		}

		if (!S_ISDIR(stbuf.st_mode))
			continue;

		strncpy(vendor_file, SYSFS_PCI_PATH, PATH_MAX);
		strncat(vendor_file, e->d_name, PATH_MAX - strlen(path));
		strncat(vendor_file, "/vendor", PATH_MAX - strlen(vendor_file));

		r = parse_sysfs_int(vendor_file, &vid);
		if (r < 0) {
			fprintf(stderr, "Can't get int value from file '%s': %s\n",
					vendor_file, strerror(errno));
			continue;
		}

		strncpy(device_file, SYSFS_PCI_PATH, PATH_MAX);
		strncat(device_file, e->d_name, PATH_MAX - strlen(path));
		strncat(device_file, "/device", PATH_MAX - strlen(vendor_file));

		r = parse_sysfs_int(device_file, &did);
		if (r < 0) {
			fprintf(stderr, "Can't get int value from file '%s': %s\n",
					device_file, strerror(errno));
			continue;
		}

		if (vid == vendor_id &&
			 	did == device_id) {
			strncpy(directory, path, directory_len);
			closedir(dir);
			return 1;
		}
	}

	closedir(dir);
	return -1;
}

