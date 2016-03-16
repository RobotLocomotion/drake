#include <stdio.h>
#include <tinydir.h>

int main(void)
{
	tinydir_dir dir;
	if (tinydir_open(&dir, ".") == -1)
	{
		perror("Error opening file");
		goto bail;
	}

	while (dir.has_next)
	{
		tinydir_file file;
		if (tinydir_readfile(&dir, &file) == -1)
		{
			perror("Error getting file");
			goto bail;
		}

		printf("%s", file.name);
		if (file.is_dir)
		{
			printf("/");
		}
		printf("\n");

		tinydir_next(&dir);
	}

bail:
	tinydir_close(&dir);
	return 0;
}
