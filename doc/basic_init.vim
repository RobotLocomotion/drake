" Make sure vim-plug is installed
if has('nvim')
    let s:vim_plug_file="~/.local/share/nvim/site/autoload/plug.vim"
    let s:vim_plug_dir="~/.local/share/nvim/plugged"
    let s:local_init_file="~/.config/nvim/local_init.vim"
    let s:local_plugins_file="~/.config/nvim/local_plugins.vim"
else
    let s:vim_plug_file="~/.vim/autoload/plug.vim"
    let s:vim_plug_dir="~/.vim/plugged"
    let s:local_init_file="~/.vim/local_init.vim"
    let s:local_plugins_file="~/.vim/local_plugins.vim"
endif
" Download vim-plug if it's not already present.
if !filereadable(expand(s:vim_plug_file))
    echom system("curl -fLo " . s:vim_plug_file . " --create-dirs "
    \ . "https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim")
endif

" Specify a directory for plugins
call plug#begin(s:vim_plug_dir)

Plug 'vim-scripts/a.vim'
Plug 'tyok/ack.vim'
Plug 'ctrlpvim/ctrlp.vim'
Plug 'vim-scripts/drawit'
Plug 'gregsexton/gitv'
Plug 'valloric/ListToggle'
Plug 'lifepillar/vim-solarized8'
Plug 'scrooloose/nerdcommenter'
Plug 'scrooloose/nerdtree'
Plug 'tyok/nerdtree-ack'
Plug 'mklabs/split-term.vim'
Plug 'tpope/tpope-vim-abolish'
Plug 'SirVer/ultisnips'
Plug 'vim-airline/vim-airline'
Plug 'vim-airline/vim-airline-themes'
Plug 'moll/vim-bbye'
Plug 'ntpeters/vim-better-whitespace'
Plug 'rhysd/vim-clang-format'
Plug 'tpope/vim-fugitive'
Plug 'weynhamz/vim-plugin-minibufexpl'
Plug 'tpope/vim-sleuth'
Plug 'wesQ3/vim-windowswap'
Plug 'valloric/YouCompleteMe'

" If you add additional "Plug 'user/repo'" lines in the file specified by
" s:local_plugins_file, those plugins will be loaded as well.
if filereadable(expand(s:local_plugins_file))
  exec "source " . s:local_plugins_file
endif

" Initialize plugin system
call plug#end()

" Set leader
let mapleader=";"

" Edit/source this file
let s:current_file=expand('<sfile>:p')
if !exists("*EditVimrc")
  function EditVimrc()
    execute 'split' s:current_file
  endfunction
endif
if !exists("*SourceVimrc")
  function SourceVimrc()
    execute 'source' s:current_file
  endfunction
endif

" This sets Solarized8 to use true Solarized colors in gVim and Neovim while
" falling back to the 256 color approximation in terminal Vim.
if (has("termguicolors"))
  set termguicolors
else
  set t_Co=256
  " Uncomment the following line if you've set up your terminal to use
  " Solarized colors.
  "let g:solarized_use16=1
endif
colorscheme solarized8_flat

nnoremap <leader>ev :call EditVimrc()<CR>
nnoremap <leader>sv :call SourceVimrc()<CR>

" Buffer configuration
set hidden
" Open next buffer in the current window.
nmap ff :bnext<CR>
" Open previous buffer in the current window.
nmap FF :bprevious<CR>

" Misc. mapings
" Make getting from INSERT to NORMAL less of a stretch.
imap jj <Esc>
" Use <leader> + navigation keys to jump between windows in both NORMAL and
" INSERT modes.
nnoremap <leader>J <c-w>j
nnoremap <leader>K <c-w>k
nnoremap <leader>H <c-w>h
nnoremap <leader>L <c-w>l
inoremap <leader>J <Esc><c-w>j
inoremap <leader>K <Esc><c-w>k
inoremap <leader>H <Esc><c-w>h
inoremap <leader>L <Esc><c-w>l

" Save the current file.
nnoremap <leader>ww :w<cr>
" Close the current file.
nnoremap <leader>qq :q<cr>
" Save and close the current file.
nnoremap <leader>wq :w<cr>:q<cr>
" Close all files.
nnoremap <leader>qa :qa<cr>

" Terminal management (Neovim only)
if has('nvim')
    " Open a new terminal in a horizontal split.
    nnoremap <leader>tj :Term<cr><C-\><C-n><c-w>x<c-w>ji
    " Open a new terminal in a vertical split.
    nnoremap <leader>tl :VTerm<cr>
    " Go from TERMINAL mode to NORMAL mode.
    tnoremap <leader>tq <C-\><C-n>
    " Go to the next tab while in TERMINAL mode.
    tnoremap <leader>gt <C-\><C-n>gt
    " Move to an adjacent window while in TERMINAL mode.
    tnoremap <leader>J <C-\><C-n><c-w>j<Esc>
    tnoremap <leader>K <C-\><C-n><c-w>k<Esc>
    tnoremap <leader>H <C-\><C-n><c-w>h<Esc>
    tnoremap <leader>L <C-\><C-n><c-w>l<Esc>
    let g:disable_key_mappings=1
    set splitright
    autocmd TermOpen * setlocal nonumber
endif

" Misc. Settings
filetype plugin on
set nohlsearch
" Use <leader><leader> as a replacement for ":".
nnoremap <leader><leader> :

" Situational awarness settings
set number
set ruler

"YCM settings
" Jump to definition.
nnoremap <leader>gg :YcmCompleter GoTo<cr>
let g:ycm_always_populate_location_list=1
" Apply fix-it suggestion.
nnoremap <leader>fi :YcmCompleter FixIt<cr>

" Toggle between header/source (Using a.vim)
nnoremap <leader>a :A<cr>

" Toggle lists (Using ListToggle)
" Toggle the location list on and off. Useful for checking YouCompleteMe
" results.
let g:lt_location_list_toggle_map = '<leader>lo'
" Toggle the quickfix list on and off.
let g:lt_quickfix_list_toggle_map = '<leader>lq'

" CtrlP settings - This provides fuzzy file opening.
let g:ctrlp_working_path_mode = 'rw'
let g:ctrlp_custom_ignore = '\v[\/](build|doc|bazel-kcov)$'
" Launch CtrlP. Use this and then start typing fragments of the file path.
nnoremap <leader>ff :CtrlP<cr>

" Airline settings
set laststatus=2
set showtabline=2
let g:airline_theme='sol'

" Enable doxygen comments
autocmd Filetype c,cpp set comments^=:///

" gitv mappings
" Launch Gitv
nnoremap <leader>gv :Gitv<cr>

" NERDTree mappings
" Launch NerdTree (file system viewer).
nnoremap <leader>nt :NERDTree<cr>
" Launch NerdTree with the current file selected.
nnoremap <leader>nf :NERDTreeFind<cr>

" Fugitive mappings
" Show the diff to HEAD for the current file.
nnoremap <leader>gd :Gdiff<cr>
" Show the git status of the repo.
nnoremap <leader>gs :Gstatus<cr>

" Fold method
set foldmethod=syntax

" Clang format settings
let g:clang_format#detect_style_file=1
" Call clang-format on the current file.
autocmd FileType c,cpp,objc nnoremap <buffer><leader>cf :<C-u>ClangFormat<CR>
" Call clang-format on the selected portion of the current file.
autocmd FileType c,cpp,objc vnoremap <buffer><leader>cf :ClangFormat<CR>

" Highlight the 80-th column.
autocmd FileType c,cpp set cc=80

" Ignore CamelCase words when spell checking
fun! IgnoreCamelCaseSpell()
  syn match CamelCase /\<[A-Z][a-z]\+[A-Z].\{-}\>/ contains=@NoSpell transparent
  syn cluster Spell add=CamelCase
endfun
autocmd BufRead,BufNewFile * :call IgnoreCamelCaseSpell()

" vim-windowswap Settings
let g:windowswap_map_keys = 0 "prevent default bindings
" Swap the contents of two windows. Press <leader>ss while in the first
" window, then navigate to the second window and press <leader>ss again.
nnoremap <silent> <leader>ss :call WindowSwap#EasyWindowSwap()<CR>

" If you put more mappings or other magic in the file specified by
" s:local_init_file, those will be added here.
if filereadable(expand(s:local_init_file))
  exec "source " . s:local_init_file
endif
