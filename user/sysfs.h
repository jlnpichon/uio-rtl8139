#ifndef SYSFS_H
#define SYSFS_H

int parse_sysfs_string(char *filename, char *string, size_t len);
int parse_sysfs_int(char *filename, uint64_t *value);

#endif /* SYSFS_H */
