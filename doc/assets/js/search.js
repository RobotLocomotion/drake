const navMenu = document.querySelector('.site-menu')
const searchIcon = document.querySelector('.search-icon')
const searchClose = document.querySelector('.search-close')
const searchInput = document.querySelector('.search input')
const searchContainer = document.querySelector('.search')
const searchResults = document.getElementById('results-container')

searchIcon.addEventListener('click', function() {
  navMenu.classList.add('search-open')
  searchContainer.classList.add('open')
  searchInput.focus()
})


searchClose.addEventListener('click', function() {
  navMenu.classList.remove('search-open')
  searchContainer.classList.remove('open')

  searchInput.value = ''
  searchResults.innerHTML = ''
})
