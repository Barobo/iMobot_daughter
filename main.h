#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <cr_section_macros.h>
#include <NXP/crp.h>

#define CR_INTEGER_PRINTF

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

#endif //MAIN_H
