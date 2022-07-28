#ifndef __JPEGENCEXIF_H__
#define __JPEGENCEXIF_H__

#include <libexif/exif-entry.h>
#include <libexif/exif-data.h>
#include <libexif/exif-ifd.h>
#include <libexif/exif-loader.h>
#include <string.h>
#include <stdlib.h>

#define EXIF_CENTER     0x0
#define EXIF_AVERAGE    0x1

#define EXIF_WB_AUTO    0xA
#define EXIF_WB_MANUAL  0xB

#define EXIF_ISO_AUTO   0x0
#define EXIF_ISO_100    0x1
#define EXIF_ISO_200    0x2
#define EXIF_ISO_400    0x3
#define EXIF_ISO_800    0x4
#define EXIF_ISO_1000   0x5
#define EXIF_ISO_1200   0x6
#define EXIF_ISO_1600   0x7

static const char ExifAsciiPrefix[] = { 0x41, 0x53, 0x43, 0x49, 0x49, 0x0, 0x0, 0x0 };

typedef struct _exif_buffer
{
  unsigned char *data;
  unsigned int size;
} exif_buffer;

void exif_buf_free (exif_buffer * buf);

exif_buffer *exif_new_buf(unsigned char *data, unsigned int size);

void exif_entry_set_string (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    const char *data);
void exif_entry_set_undefined (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    exif_buffer * buf);

void exif_entry_set_byte (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    ExifByte n);
void exif_entry_set_short (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    ExifShort n);
void exif_entry_set_long (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    ExifLong n);
void exif_entry_set_rational (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    ExifRational r);

void exif_entry_set_sbyte (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    ExifSByte n);
void exif_entry_set_sshort (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    ExifSShort n);
void exif_entry_set_slong (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    ExifSLong n);
void exif_entry_set_srational (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag,
    ExifSRational r);

/* SP: Function to remove tags with the same interface as the other
 * functions from here.
 */
void exif_entry_unset(ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag);

/* SP: GPS setters.  */
void exif_entry_set_gps_longitude(ExifData * pEdata, ExifRational r1, ExifRational r2, ExifRational r3);
void exif_entry_set_gps_longitude_ref_east(ExifData * pEdata);
void exif_entry_set_gps_longitude_ref_west(ExifData * pEdata);

void exif_entry_set_gps_latitude(ExifData * pEdata, ExifRational r1, ExifRational r2, ExifRational r3);
void exif_entry_set_gps_latitude_ref_north(ExifData * pEdata);
void exif_entry_set_gps_latitude_ref_south(ExifData * pEdata);

void exif_entry_set_gps_altitude(ExifData * pEdata, ExifRational r1);
void exif_entry_set_gps_altitude_ref_above_sea_level(ExifData * pEdata);
void exif_entry_set_gps_altitude_ref_below_sea_level(ExifData * pEdata);

void exif_entry_set_gps_version(ExifData * pEdata, ExifByte r1, ExifByte r2, ExifByte r3, ExifByte r4);
void exif_entry_set_gps_dop(ExifData * pEdata, ExifRational r1);

void exif_entry_set_gps_img_direction(ExifData * pEdata, ExifRational r1);
void exif_entry_set_gps_img_direction_ref_true(ExifData * pEdata);
void exif_entry_set_gps_img_direction_ref_magnetic(ExifData * pEdata);

void exif_entry_set_gps_track(ExifData * pEdata, ExifRational r1);
void exif_entry_set_gps_track_ref_true(ExifData * pEdata);
void exif_entry_set_gps_track_ref_magnetic(ExifData * pEdata);

void exif_entry_set_gps_speed(ExifData * pEdata, ExifRational r1);
void exif_entry_set_gps_speed_ref_kilometers(ExifData * pEdata);
void exif_entry_set_gps_speed_ref_miles(ExifData * pEdata);
void exif_entry_set_gps_speed_ref_knots(ExifData * pEdata);


/* SP: Generic GPS setters. */
void exif_entry_set_gps_byte1(ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag, ExifByte r1);
void exif_entry_set_gps_rational1(ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag, ExifRational r1);
void exif_entry_set_gps_rational3(ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag, ExifRational r1, ExifRational r2, ExifRational r3);
void exif_entry_set_gps_string (ExifData * pEdata, ExifIfd eEifd, ExifTag eEtag, const char *s);


#endif
