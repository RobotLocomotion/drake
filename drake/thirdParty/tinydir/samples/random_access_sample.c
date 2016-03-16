#include <stdio.h>
#include <tinydir.h>

int main(void)
{
	tinydir_dir dir;
	size_t i;
	if (tinydir_open_sorted(&dir, ".") == -1)
	{
		perror("Error opening file");
		goto bail;
	}

	for (i = 0; i < dir.n_files; i++)
	{
		tinydir_file file;
		if (tinydir_readfile_n(&dir, &file, i) == -1)
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
	}

bail:
	tinydir_close(&dir);
	return 0;
}
