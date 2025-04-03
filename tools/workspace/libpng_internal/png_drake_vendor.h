#pragma once

/* The following are Drake's adjustments to libpng. */

/* Always use the macros, never the slow out-of-line functions. */
#undef PNG_READ_INT_FUNCTIONS_SUPPORTED

/* Suppress benign eXIf warnings, as they produce unnecessary test output. */
#undef PNG_READ_eXIf_SUPPORTED
