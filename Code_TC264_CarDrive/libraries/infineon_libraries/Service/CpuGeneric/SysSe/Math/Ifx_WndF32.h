#ifndef IFX_WNDF32_H
#define IFX_WNDF32_H

#include "Ifx_Cf32.h"

#define IFX_WNDF32_TABLE_LENGTH (1024)

/** \addtogroup library_srvsw_sysse_math_f32_wnd
 * \{ */

IFX_EXTERN CONST_CFG float32 Ifx_g_WndF32_hannTable[IFX_WNDF32_TABLE_LENGTH / 2];
IFX_EXTERN CONST_CFG float32 Ifx_g_WndF32_blackmanHarrisTable[IFX_WNDF32_TABLE_LENGTH / 2];

/** \brief Apply a window to complex array x which has nX length.
 *
 * Example: Ifx_WndF32_apply(x, Ifx_g_WndF32_hannTable, 1024); */
IFX_INLINE void Ifx_WndF32_apply(cfloat32 *x, CONST_CFG float32 *window, short nX)
{
    VecWin_f32(&x->real, window, nX, IFX_WNDF32_TABLE_LENGTH, 2, 1);
}


/** \} */

#endif /* IFX_WNDF32_H */
