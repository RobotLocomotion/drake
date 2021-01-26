
(function ($) {
    'use strict';



    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Navigation

    // Global vars
    var navTarget = $('body').attr('data-page-url');
    var docTitle = document.title;
    var History = window.History;

    // State change event
    History.Adapter.bind(window, 'statechange', function () {
        var state = History.getState();
        // console.log(state);

        // Loading state
        $('body').addClass('loading');

        // Load the page
        $('.page-loader').load(state.hash + ' .page__content', function () {

            // Scroll to top
            $('body, html').animate({
                scrollTop: 0
            }, 300);

            // Find transition time
            var transitionTime = 400;

            // After current content fades out
            setTimeout(function () {

                // Remove old content
                $('.page .page__content').remove();

                // Append new content
                $('.page-loader .page__content').appendTo('.page');

                // Set page URL
                $('body').attr('data-page-url', window.location.pathname);

                // Update navTarget
                navTarget = $('body').attr('data-page-url');

                // Set page title
                docTitle = $('.page__content').attr('data-page-title');
                document.title = docTitle;

                // Run page functions
                pageFunctions();

            }, transitionTime);

        });

    });


    // On clicking a link

    if ($('body').hasClass('ajax-loading')) {

        $(document).on('click', 'a', function (event) {

            // Don't follow link
            event.preventDefault();

            // Get the link target
            var thisTarget = $(this).attr('href');

            // If we don't want to use ajax, or the link is an anchor/mailto/tel
            if ($(this).hasClass('js-no-ajax') || thisTarget.indexOf('#') >= 0 || thisTarget.indexOf('mailto:') >= 0 || thisTarget.indexOf('tel:') >= 0) {

                // Use the given link
                window.location = thisTarget;
            }

            // if it's a contact modal
            else if ($(this).hasClass('js-contact')) {

                // Open contact modal
            }

            // If link is handled by some JS action – e.g. fluidbox
            else if ($(this).is('.gallery__item__link')) {

                // Let JS handle it
            }

            // If link is external
            else if (thisTarget.indexOf('http') >= 0) {

                // Go to the external link
                window.open(thisTarget, '_blank');

            }

            // If link is internal
            else {

                // Change navTarget
                navTarget = thisTarget;

                // Switch the URL via History
                History.pushState(null, docTitle, thisTarget);
            }

        });

    }



    // Modals
    function lockPage() {
        $('.page').addClass('locked');
        $('body').addClass('locked');
    }

    function unlockPage() {
        $('.page').removeClass('locked');
        $('body').removeClass('locked');
    }

    $(document).on('click', '.js-contact', function (event) {
        event.preventDefault();

        $('body').removeClass('menu--open');
        $('.contact').addClass('visible');
        lockPage();

        $('.button--close-modal').on('click', function () {
            $('.contact').removeClass('visible');
            unlockPage();
        });
    });



    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Page load

    function pageFunctions() {


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Show content

        // Show the content
        $('body').removeClass('loading');

        // Hide the menu
        $('body').removeClass('menu--open');



        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Active links

        // Switch active link states
        $('.active-link').removeClass('active-link');

        $('a[href="' + navTarget + '"]').addClass('active-link');


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Images

        $('.post__content p > img').each(function () {
            var thisP = $(this).parent('p');
            $(this).insertAfter(thisP);
            $(this).wrapAll('<div class="image-wrap"></div>');
            thisP.remove();
        });



        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Videos

        // For each iframe
        $('.post__content iframe').each(function () {

            // If it's YouTube or Vimeo
            if ($(this).attr('src').indexOf('youtube') >= 0 || $(this).attr('src').indexOf('vimeo') >= 0) {

                var width = $(this).attr('width');
                var height = $(this).attr('height');
                var ratio = (height / width) * 100;

                // Wrap in video container
                $(this).wrapAll('<div class="video" style="padding-bottom:' + ratio + '%;"></div>');

            }

        });


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Tables

        $('.post__content table').each(function () {
            $(this).wrapAll('<div class="table-wrap"></div>');
        });

    }

    // Run functions on load
    pageFunctions();


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Menu

    $(document).on('click', '.js-menu-toggle', function () {

        // If already open
        if ($('body').hasClass('menu--open')) {
            $('body').removeClass('menu--open');
        }

        // If not open
        else {
            $('body').addClass('menu--open');
        }
    });

    $(document).on('click', '.menu__list__item__link', function () {

        // If menu is open when you click a link on mobile
        if ($('.menu').hasClass('menu--open')) {
            $('.menu').removeClass('menu--open');
        }
    });



    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Listing post click

    // Click anywhere on the post to go to the link
    $(document).on('click', '.post', function () {

        var targetPost = $(this).find('.post__title a').attr('href');

        if ($('body').hasClass('ajax-loading')) {

            // Change navTarget
            navTarget = targetPost;

            // Switch the URL via History
            History.pushState(null, docTitle, targetPost);
        }

        else {
            // Use the given link
            window.location = targetPost;
        }
    });



    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Contact Form

    // Override the submit event
    $(document).on('submit', '#contact-form', function (e) {

        // Clear previous classes
        $('.contact-form__item--error').removeClass('contact-form__item--error');

        // Get form elements
        var emailField = $('.contact-form__input[name="email"]');
        var nameField = $('.contact-form__input[name="name"]');
        var messageField = $('.contact-form__textarea[name="message"]');
        var gotchaField = $('.contact-form__gotcha');

        // Validate email
        if (emailField.val() === '') {
            emailField.closest('.contact-form__item').addClass('contact-form__item--error');
        }

        // Validate name
        if (nameField.val() === '') {
            nameField.closest('.contact-form__item').addClass('contact-form__item--error');
        }

        // Validate message
        if (messageField.val() === '') {
            messageField.closest('.contact-form__item').addClass('contact-form__item--error');
        }

        // If all fields are filled, except gotcha
        if (emailField.val() !== '' && nameField.val() !== '' && messageField.val() !== '' && gotchaField.val().length === 0) {

            // Submit the form!
        }

        else {

            // Stop submission
            e.preventDefault();
        }

    });




}(jQuery));
