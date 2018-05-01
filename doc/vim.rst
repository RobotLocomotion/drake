.. _vim_neovim_notes:

*****************************************
Vim/Neovim Notes
*****************************************

.. contents:: `Table of contents`
   :depth: 3
   :local:

.. _vim-intro:

Introduction
============

This page contains notes on ways to use `Vim <https://www.vim.org/>`_ or
`Neovim <https://neovim.io/>`_ with Drake. Wherever there are differences
between the two the Neovim value will be given in parentheses after the Vim
value (e.g. ``.vimrc`` (``.config/nvim/init.vim``)).

Installing Neovim
=================

Follow the instructions on the Neovim wiki
(`Ubuntu <https://github.com/neovim/neovim/wiki/Installing-Neovim#ubuntu>`_,
`Homebrew <https://github.com/neovim/neovim/wiki/Installing-Neovim#homebrew-macos--linuxbrew-linux>`_).
Ubuntu users should make sure that they install the Python modules as well.

Configuration Files
===================

Save ``basic_init.vim`` (:download:`download <basic_init.vim>`) to ``~/.vimrc``
(``~/.config/nvim/init.vim``). This file sets up some plugins that we
have found useful for working on Drake. Feel free to modify your copy!

Now we need to setup the plugins specified in ``.vimrc`` (``init.vim``). Open a
terminal and run vim. (NOTE: You will get an error message about the color
scheme not being found. This is because the plugin that supplies the color
scheme hasn't been installed yet. You're about to install it now.) In vim, type
``:PlugInstall`` and press Enter. The plugin manager (`VimPlug
<https://github.com/junegunn/vim-plug>`_) will download the plugins from
GitHub. You may see an error message from from YouCompleteMe. If you do, run
``:PlugInstall`` again and it should go away.

Setting up Code-Completion/Linting/Jump To Declaration
======================================================
`YouCompleteMe <https://github.com/Valloric/YouCompleteMe>`_ is a plugin that
provides realtime linting + jump-to-declaration functionality. It has a
compiled component, so we need to build it now. This step is only necessary
when the version of YouCompleteMe changes. First, find where VimPlug installs
your packages by running ``:echo g:plug_home`` in Vim. Then, in a terminal,
run::

    cd <result of previous command>/YouCompleteMe
    git submodule update --init --recursive
    ./install.py --clang-completer

In order for YouCompleteMe to provide code-completion and linting for Drake's
C++ files, it needs to know what flags to use when compiling them. Fortunately,
`bazel-compilation-database
<https://github.com/grailbio/bazel-compilation-database>`_ provides a way for
YouCompleteMe to get this information from Bazel. The following steps should
get this working in Drake:

   1. Download the bazel-compilation-database 0.2.2
      `tarball <https://github.com/grailbio/bazel-compilation-database/archive/0.2.2.tar.gz>`_.
   2. Extract the contents of the tarball to some directory. We will refer to
      this directory as ``<bcd-location>``.
   3. From the root of your Drake repository run::

         mkdir bazel
         ln -s <bcd-location> bazel/compilation_database

   4. Add the following line to your ``.vimrc`` (``init.vim``)::

         let g:ycm_global_ycm_extra_conf = '<bcd-location>/.ycm_extra_conf.py'

Now, when you open a Drake C++ file, you should get real-time linting,
completion, and jump-to-declaration functionality. If it appears not to be
working, try running `:YcmDiags` to force YouCompleteMe to compile the current
file.

Additional Mappings
===================

The attached ``basic_init.vim`` file pulls in several plugins and defines some
mappings for working with them. We've tried to comment each of these in the
file. Here are some highlights. A lot of these start with <leader>, which in
the attached file is mapped to ";" (so <leader>gv becomes ;gv). You can, of
course, change <leader> to whatever you prefer in your ``.vimrc`` (``init.vim``) file.

Navigation Mappings
-------------------
Note that these use capital H, J, K, and L.

 - <leader>H switch to the window to the left of the current window.
 - <leader>J switch to the window below the current window.
 - <leader>K switch to the window above the current window.
 - <leader>L switch to the window to the right of  the current window.

Git Mappings
------------
These mappings come use `fugitive.git <https://github.com/tpope/vim-fugitive>`_ and
`Gitv <https://github.com/gregsexton/gitv>`_ to perform Git-related tasks.

 - <leader>gd shows the git diff for the current file. You can move hunks into
   the index using Vim's diff commands.
 - <leader>gs shows the current git status using fugitive.git.You can stage
   files by moving the cursor to the line and pressing "-". Press cc to start
   editing the commit message. Save and exit in the commit message to commit.
 - <leader>gv launches Gitv, a Gitk clone for Vim. You can use this to inspect
   commits, switch branches, cherry-pick, and more

Linting and Jump-to-Declaration Mappings
----------------------------------------
These mappings use the aforementioned YouCompleteMe.

 - <leader>gg jumps to the declaration of the class/function under the cursor
   (mnemonic "go go!"). Use the standard jump command Ctrl+o to go back.
 - <leader>fi applies the "fix-it" action listed at the bottom of the buffer
   (mnemonic "fix it!").
 - <leader>lo toggles the Location List on and off (mnemonic "LOcation"). All
   errors found by YouCompleteMe are listed in the location list, which can
   make it easy to go from one to the next.

File Tree Explorer Mappings
---------------------------
These mappings use `NERDTree <https://github.com/scrooloose/nerdtree>`_ to
provide a file tree explorer in Vim.

 - <leader>nt launches NERDTree for the current working directory
 - <leader>nf launches NERDTree with the current file selected. This is
   particularly useful for getting to the appropriate ``BUILD.bazel`` file for a
   given source file.

Fuzzy Finder Mappings
---------------------
These mappings use `ctrlp.vim <https://github.com/ctrlpvim/ctrlp.vim>`_ to
provide fuzzy file lookup in Vim.

 - <leader>ff launches CtrlP in file mode (mnemonic "find file"). Start typing
   any portions of the file path (they don't have to be contiguous!) and select
   the file you want once the choices are narrowed down sufficiently.
