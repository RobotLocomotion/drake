---
title: Vim/Neovim Notes
---


# Introduction


This page contains notes on ways to use [Vim](https://www.vim.org/) or
[Neovim](https://neovim.io/) with Drake. Wherever there are differences
between the two the Neovim value will be given in curly braces after the Vim
value (e.g. ``.vimrc`` {``.config/nvim/init.vim``}).

# Installing Neovim

If you want to use Neovim, follow the instructions on the Neovim wiki
([Ubuntu](https://github.com/neovim/neovim/wiki/Installing-Neovim#ubuntu),
[Homebrew](https://github.com/neovim/neovim/wiki/Installing-Neovim#homebrew-macos--linuxbrew-linux)).
Ubuntu users should make sure that they install the Python modules as well.

# Configuration Files

``sample_vimrc`` ([download](/sample_vimrc)) provides a sample
configuration file that sets up some plugins and key-mappings that users may
find useful for working with Drake. It should work out of the box if copied
to ``~/.vimrc`` {``~/.config/nvim/init.vim``}. Be sure to back up any existing
``.vimrc`` {``init.vim``} before copying! Users with existing Vim
configurations are, of course, free to cherry-pick whatever portions of
``sample_vimrc`` they'd like.

If you do copy ``sample_vimrc`` to ``.vimrc`` {``init.vim``}, your next step
is to install the plugins that it specifies.

1. Open a terminal and launch Vim. The plugin manager
   ([vim-plug](https://github.com/junegunn/vim-plug)) will be automatically
   installed (to disable this behavior, delete the four lines following the
   comment "Remove these lines ...").
2. In Vim, type ``:PlugInstall`` and press "Enter". vim-plug will download
   the specified plugins from GitHub. If you later decide to stop using
   vim-plug, you can remove it along with the plugins it installs by
   deleting the following
      * ``~/.vim/autoload/plug.vim``
         {``~/.local/share/nvim/site/autoload/plug.vim``}
      * ``~/.vim/plugged`` {``~/.local/share/nvim/plugged``}

# Setting up Code-Completion/Linting/Jump To Declaration

[YouCompleteMe](https://github.com/Valloric/YouCompleteMe) is a plugin that
provides real-time linting + jump-to-declaration functionality. It has a
compiled component, which needs to be built before the plugin can be used.

1. Let ``<YouCompleteMe-location>`` be the directory in which the plugin is
  located. If you're using the provided ``sample_vimrc``, you can determine
  ``<YouCompleteMe-location>`` by running ``:echo g:plug_home`` in Vim and
  appending ``/YouCompleteMe`` to the result.
2. In a terminal, run
```
cd <YouCompleteMe-location>
git submodule update --init --recursive
./install.py --clang-completer
```

In order for YouCompleteMe to provide code-completion and linting for Drake's
C++ files, it needs to know what flags to use when compiling them. Fortunately,
[bazel-compilation-database](https://github.com/grailbio/bazel-compilation-database)
provides a way for YouCompleteMe to get this information from Bazel. The
following steps should get this working in Drake:

1. Download the bazel-compilation-database 0.2.2
  [tarball](https://github.com/grailbio/bazel-compilation-database/archive/0.2.2.tar.gz).
2. Extract the contents of the tarball to some directory. We will refer to
  this directory as ``<bcd-location>``.
3. From the root of your Drake repository run:
```
mkdir bazel
ln -s <bcd-location> bazel/compilation_database
```
4. Add the following line to your ``.vimrc`` {``init.vim``}:
```
let g:ycm_global_ycm_extra_conf = '<bcd-location>/.ycm_extra_conf.py'
```

Now, when you open a Drake C++ file, you should get real-time linting and
completion, as well as jump-to functionality (through the ``:YcmCompeter GoTo``
command). If it appears not to be working, try running ``:YcmDiags`` to force
YouCompleteMe to compile the current file. Note that for YouCompleteMe to see
a given Drake header file, the corresponding Bazel target must have been built.
See also, [Linting and Jump-to-Declaration Mappings](#linting-and-jump-to-declaration-mappings).

# Additional Mappings

The aforementioned ``sample_vimrc`` file pulls in several plugins and defines
mappings for working with them. It also defines some general-purpose mappings.
Here are some highlights. A lot of these start with ``<leader>``, which in the
attached file is mapped to ``;`` (so ``<leader>gv`` becomes ``;gv``). You can, of course,
change ``<leader>`` to whatever you prefer in your ``.vimrc`` {``init.vim``} file.

## Git Mappings

These mappings use [fugitive.git](https://github.com/tpope/vim-fugitive) and
[Gitv](https://github.com/gregsexton/gitv) to perform Git-related tasks.

* ``<leader>gd`` shows the git diff for the current file. You can move hunks into
   the index using Vim's diff commands.
* ``<leader>gs`` shows the current git status using fugitive.git.You can stage
   files by moving the cursor to the line and pressing "-". Press cc to start
   editing the commit message. Save and exit in the commit message to commit.
* ``<leader>gv`` launches gitv, a Gitk clone for Vim. You can use this to inspect
   commits, switch branches, cherry-pick, and more

## Linting and Jump-to-Declaration Mappings

These mappings use [YouCompleteMe](https://github.com/Valloric/YouCompleteMe)
and [ListToggle](https://github.com/Valloric/ListToggle).

* ``<leader>gg`` jumps to the declaration of the class/function under the cursor
   (mnemonic "go go!"). Use the standard jump command Ctrl+o to go back.
* ``<leader>fi`` applies the "fix-it" action listed at the bottom of the buffer
   (mnemonic "fix it!").
* ``<leader>lo`` toggles the Location List on and off (mnemonic "LOcation"). All
   errors found by YouCompleteMe are listed in the location list, which can
   make it easy to go from one to the next.

## File Tree Explorer Mappings

These mappings use [NERDTree](https://github.com/scrooloose/nerdtree) to
provide a file tree explorer in Vim.

* ``<leader>nt`` launches NERDTree for the current working directory
* ``<leader>nf`` launches NERDTree with the current file selected. This is
   particularly useful for getting to the appropriate ``BUILD.bazel`` file for a
   given source file.

## Fuzzy Finder Mappings

These mappings use [ctrlp.vim](https://github.com/ctrlpvim/ctrlp.vim) to
provide fuzzy file lookup in Vim.

* ``<leader>ff`` launches CtrlP in file mode (mnemonic "find file"). Start typing
   any portions of the file path (they don't have to be contiguous!) and select
   the file you want once the choices are narrowed down sufficiently.
