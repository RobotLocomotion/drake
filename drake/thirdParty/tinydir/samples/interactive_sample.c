#include <stdio.h>
#include <tinydir.h>

int main(void)
{
	tinydir_dir dir;
	if (tinydir_open_sorted(&dir, ".") == -1)
	{
		perror("Error opening file");
		goto bail;
	}

	for (;;)
	{
		size_t i;
		char input[256];
		for (i = 0; i < dir.n_files; i++)
		{
			tinydir_file file;
			if (tinydir_readfile_n(&dir, &file, i) == -1)
			{
				perror("Error getting file");
				goto bail;
			}

			if (file.is_dir)
			{
				printf("[%u] ", (unsigned int)i);
			}
			printf("%s", file.name);
			if (file.is_dir)
			{
				printf("/");
			}
			printf("\n");
		}
		printf("?");

		if (fgets(input, 256, stdin) == NULL)
		{
			break;
		}
		else
		{
			int choice = atoi(input);
			if (choice >= 0 && (size_t)choice < dir.n_files)
			{
				if (tinydir_open_subdir_n(&dir, choice) == -1)
				{
					perror("Error opening subdirectory");
					goto bail;
				}
			}
		}
	}

bail:
	tinydir_close(&dir);
	return 0;
}
