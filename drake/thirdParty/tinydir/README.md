TinyDir
=======
[![Build Status](https://travis-ci.org/cxong/tinydir.svg?branch=master)](https://travis-ci.org/cxong/tinydir)[![Project Stats](https://www.openhub.net/p/tinydir/widgets/project_thin_badge.gif)](https://www.openhub.net/p/tinydir)

Lightweight, portable and easy to integrate C directory and file reader. TinyDir wraps dirent for POSIX and FindFirstFile for Windows.

Example
=======

There are two methods. Error checking omitted:

```C
tinydir_dir dir;
tinydir_open(&dir, "/path/to/dir");

while (dir.has_next)
{
	tinydir_file file;
	tinydir_readfile(&dir, &file);

	printf("%s", file.name);
	if (file.is_dir)
	{
		printf("/");
	}
	printf("\n");

	tinydir_next(&dir);
}

tinydir_close(&dir);
```

```C
tinydir_dir dir;
int i;
tinydir_open_sorted(&dir, "/path/to/dir");

for (i = 0; i < dir.n_files; i++)
{
	tinydir_file file;
	tinydir_readfile_n(&dir, &file, i);

	printf("%s", file.name);
	if (file.is_dir)
	{
		printf("/");
	}
	printf("\n");
}

tinydir_close(&dir);
```

See the `/samples` folder for more examples, including an interactive command-line directory navigator.

Language
========

ANSI C, or C90.

Platforms
=========

POSIX and Windows supported. Open to the possibility of supporting other platforms.

License
=======

Simplified BSD.

Known Limitations
=================

- Limited path and filename sizes
- No wide char support
