/*
Enables clicking for the mobile "hamburger" (three-line) menu item.
*/

const siteHeader = document.querySelector('.site-header')
const mobileButton = document.querySelector('.menu-mobile-toggle')
const body = document.querySelector('body')

mobileButton.addEventListener('click', function(event) {
  siteHeader.classList.toggle('open');
  body.classList.toggle('overflow-hidden');
})
