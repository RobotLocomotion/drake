#pragma once

/* The following are Drake's adjustments to libpng. */

/* Always use the macros, never the slow out-of-line functions. */
#undef PNG_READ_INT_FUNCTIONS_SUPPORTED
