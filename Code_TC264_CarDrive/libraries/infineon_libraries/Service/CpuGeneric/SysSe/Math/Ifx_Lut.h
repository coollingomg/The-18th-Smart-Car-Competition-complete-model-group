#ifndef IFX_LUT_H
#define IFX_LUT_H
//________________________________________________________________________________________
#include "Cpu/Std/IfxCpu_Intrinsics.h"
//________________________________________________________________________________________
#ifndef IFX_LUT_TABLE_CONST
/** \brief 0 = Lookup tables are generated and stored in RAM,
 * 1 = Lookup tables are precompiled and stored in FLASH */
#define IFX_LUT_TABLE_CONST (1)
#endif

#if (IFX_LUT_TABLE_CONST != 0)
/** \brief Lookup table type modifier (stored in ROM) */
#   define IFX_LUT_TABLE    const
#else
/** \brief Lookup table type modifier (stored in RAM) */
#   define IFX_LUT_TABLE
#endif

/** \brief Define the resolution (in bits) of cosinus and sinus table \ingroup library_srvsw_sysse_math_lut */
#define IFX_LUT_ANGLE_BITS       (12)

/** \brief Define the cosinus and sinus table size \ingroup library_srvsw_sysse_math_lut */
#define IFX_LUT_ANGLE_RESOLUTION (1 << IFX_LUT_ANGLE_BITS)

/** \brief Value used as 180 degree angle */
#define IFX_LUT_ANGLE_PI         (IFX_LUT_ANGLE_RESOLUTION / 2)

//________________________________________________________________________________________
/** \brief Fixed-point angle type.
 *
 * Resolution is defined by \ref IFX_LUT_ANGLE_RESOLUTION */
typedef sint32 Ifx_Lut_FxpAngle;

/** \brief Convert from float32 angle in radian into fixed-point angle type */
#define IFX_LUT_F32_TO_FXPANGLE(f)  ((Ifx_Lut_FxpAngle)(f / IFX_PI * IFX_LUT_ANGLE_PI))

/** \brief Convert from fixed-point angle type to float32 angle in radian */
#define IFX_LUT_FXPANGLE_TO_F32(fx) ((float32)fx * IFX_PI / IFX_LUT_ANGLE_PI)

//________________________________________________________________________________________
#if (1U << IFX_LUT_ANGLE_BITS) != IFX_LUT_ANGLE_RESOLUTION
#error (1U << IFX_LUT_ANGLE_BITS) != IFX_LUT_ANGLE_RESOLUTION
#endif
//________________________________________________________________________________________
#endif /* IFX_LUT_H */
