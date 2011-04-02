#include <string.h>
#include "consoleprint.h"

/*
 * '__CR_SEMIHOST' symbol picked up by Code Red tools when image loaded to enable
 * semihosting support (when auto option is selected).
 */
extern void __CR_SEMIHOST () __attribute__ ((weak, alias ("consoleprint")));

/*
 * Prototype for Code Red Redlib semihosting stub function that carries out
 * semihosting "SYS_WRITE" operation. Note that Code Red makes no
 * guarantees that the interface to this stub function will not
 * change in future versions of the tools.
 */
#define LIBSTUB_SYS_WRITE __write
int LIBSTUB_SYS_WRITE (int, char *, int);

/*
 * consoleprint() -  provides a "print string to console" function that uses
 * the CodeRed semihosting debug channel functionality. Because this can
 * send a full string in one operation, rather than a single character (as
 * using printf will) this will provide faster prints to the console.
 *
 * Input - pointer to zero-terminated character string
 * Returns 0 if successful
 *
 * Note that in order to consoleprint() to actually display to the console,
 * the semihosting interface using stdout must have been setup. The defining
 * of the symbol '__CR_SEMIHOST' should force the Code Red debugger to do
 * this.
 */
int consoleprint(char *cpstring)
{
	 int slen, res;
	 // Calculate length of string
	 slen = strlen (cpstring);
	 // Call function to carry out semihosting "SYS_WRITE" operation
	 res = LIBSTUB_SYS_WRITE (0, cpstring,slen);	// Note that '0' represents stdout
	 return res;
}
