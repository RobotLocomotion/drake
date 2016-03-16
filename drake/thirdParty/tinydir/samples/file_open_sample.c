#include <stdio.h>
#include <tinydir.h>

int main(int argc, char *argv[])
{
	tinydir_file file;
	if (argc != 2)
	{
		fprintf(stderr, "Usage: test filename\n");
		return 1;
	}
	if (tinydir_file_open(&file, argv[1]) == -1)
	{
		perror("Error opening file");
		return 1;
	}
	printf("Path: %s\nName: %s\nExtension: %s\nIs dir? %s\nIs regular file? %s\n",
		   file.path, file.name, file.extension,
		   file.is_dir?"yes":"no", file.is_reg?"yes":"no");
	return 0;
}
