/*
 * write-exif.c
 *
 * Placed into the public domain by Daniel Fandrich
 *
 * Create a new EXIF data block and write it into a JPEG image file.
 *
 * The JPEG image data used in this example is fixed and is guaranteed not
 * to contain an EXIF tag block already, so it is easy to precompute where
 * in the file the EXIF data should be. In real life, a library like
 * libjpeg (included with the exif command-line tool source code) would
 * be used to write to an existing JPEG file.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <libexif/exif-data.h>
#include <libjpeg/jpeg-data.h>
#include <JpegEncoderEXIF/JpegEncoderEXIF.h>


/* byte order to use in the EXIF block */
#define FILE_BYTE_ORDER EXIF_BYTE_ORDER_INTEL

/* comment to write into the EXIF block */
#define FILE_COMMENT "libexif demonstration image"

/* special header required for EXIF_TAG_USER_COMMENT */
#define ASCII_COMMENT "ASCII\0\0\0"

static ExifEntry *create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag, size_t len)
{
    void *buf;
    ExifEntry *entry;

    /* Create a memory allocator to manage this ExifEntry */
    ExifMem *mem = exif_mem_new_default();
    assert(mem != NULL); /* catch an out of memory condition */

    /* Create a new ExifEntry using our allocator */
    entry = exif_entry_new_mem (mem);
    assert(entry != NULL);

    /* Allocate memory to use for holding the tag data */
    buf = exif_mem_alloc(mem, len);
    assert(buf != NULL);

    /* Fill in the entry */
    entry->data = (unsigned char*)buf;
    entry->size = len;
    entry->tag = tag;
    entry->components = len;
    entry->format = EXIF_FORMAT_UNDEFINED;

    /* Attach the ExifEntry to an IFD */
    exif_content_add_entry (exif->ifd[ifd], entry);

    /* The ExifMem and ExifEntry are now owned elsewhere */
    exif_mem_unref(mem);
    exif_entry_unref(entry);

    return entry;
}

int main(int argc, char **argv)
{

    ExifEntry *entry;

    //Input JPG
    char mInputFilename[]="/data/exifyay/build/example.jpg";
    printf("file: %s\n", mInputFilename);

    //Load JPG
    JPEGData * mJpegData = jpeg_data_new_from_file(mInputFilename);
    printf("mJpegData %d\n", (int)mJpegData);

    //Load Exif data from JPG
    ExifData * mExifData = jpeg_data_get_exif_data(mJpegData);
    printf("mExifData %d\n", (int)mExifData);

    if (mExifData == NULL)
    {
        mExifData = exif_data_new();
    }


    // python command for example
    // "exiftool -GPSLatitude=\"%s\" -GPSLongitude=\"%s\" -GPSLongitudeRef=\"%s\" -GPSLatitudeRef=\"%s\" -GPSAltitudeRef=\'Above Sea level\' -GPSAltitude=\"%s\" %s*"

    //Set some Exif options
    exif_data_set_option(mExifData, EXIF_DATA_OPTION_FOLLOW_SPECIFICATION);
    exif_data_set_data_type(mExifData, EXIF_DATA_TYPE_COMPRESSED);
    exif_data_set_byte_order(mExifData, FILE_BYTE_ORDER);

    /* example text */
    entry = create_tag(mExifData, EXIF_IFD_EXIF, EXIF_TAG_USER_COMMENT, 
            sizeof(ASCII_COMMENT) + sizeof(FILE_COMMENT) - 2);
    /* Write the special header needed for a comment tag */
    memcpy(entry->data, ASCII_COMMENT, sizeof(ASCII_COMMENT)-1);
    /* Write the actual comment text, without the trailing NUL character */
    memcpy(entry->data+8, FILE_COMMENT, sizeof(FILE_COMMENT)-1);
    /* create_tag() happens to set the format and components correctly for
     * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */

    /* Create a EXIF_TAG_SUBJECT_AREA tag */
    entry = create_tag(mExifData, EXIF_IFD_EXIF, EXIF_TAG_SUBJECT_AREA,
               4 * exif_format_get_size(EXIF_FORMAT_SHORT));
    entry->format = EXIF_FORMAT_SHORT;
    entry->components = 4;

    /* GPS Data */
    double dlat = 37.667788; // we supposed that your GPS data is Double if its not skip this step
    double dlon = -122.667788; // we supposed that your GPS data is Double if its not skip this step
    double dalt = 5.4321;

    /* GPS Lat */
    {
        // create our latitude tag, the whole  field is 24 bytes long
        entry = create_tag(mExifData, EXIF_IFD_GPS, EXIF_TAG_GPS_LATITUDE, 24);

        // Set the field's format and number of components, this is very important!
        entry->format = EXIF_FORMAT_RATIONAL;
        entry->components = 3;

        // convert double to unsigned long array
        double coord = fabs(dlat);
        double sec = (coord * 3600);
        int deg = sec / 3600;
        sec = sec - 3600.0*((int)sec/3600);
        int min = sec / 60;
        sec = sec - 60.0*((int)sec/60);
        // printf("debug sec %f\n", sec);

        ExifRational secr;
        secr.numerator = (unsigned)(sec * 1000000.0);
        secr.denominator = (unsigned)1000000 ;

        ExifRational degr;
        degr.numerator = (unsigned)(deg * 1000000.0);
        degr.denominator = (unsigned)1000000 ;

        ExifRational minr;
        minr.numerator = (unsigned)(min * 1000000.0);
        minr.denominator = (unsigned)1000000 ;

        exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, degr);
        exif_set_rational(entry->data+sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, minr);
        exif_set_rational(entry->data+2*sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, secr);
    }

    /* GPS Lon */
    {
        // create our latitude tag, the whole  field is 24 bytes long
        entry = create_tag(mExifData, EXIF_IFD_GPS, EXIF_TAG_GPS_LONGITUDE, 24);

        // Set the field's format and number of components, this is very important!
        entry->format = EXIF_FORMAT_RATIONAL;
        entry->components = 3;
        // convert double to unsigned long array
        double coord = fabs(dlon);
        double sec = (coord * 3600);
        int deg = sec / 3600;
        sec = sec - 3600.0*((int)sec/3600);
        int min = sec / 60;
        sec = sec - 60.0*((int)sec/60);
        // printf("debug sec %f\n", sec);

        ExifRational secr;
        secr.numerator = (unsigned)(sec * 1000000.0);
        secr.denominator = (unsigned)1000000 ;

        ExifRational degr;
        degr.numerator = (unsigned)(deg * 1000000.0);
        degr.denominator = (unsigned)1000000 ;

        ExifRational minr;
        minr.numerator = (unsigned)(min * 1000000.0);
        minr.denominator = (unsigned)1000000 ;

        exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, degr);
        exif_set_rational(entry->data+sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, minr);
        exif_set_rational(entry->data+2*sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, secr);
    }

    /* GPS Alt */
    {
        // create our latitude tag, the whole  field is 24 bytes long
        entry = create_tag(mExifData, EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE, 8);

        // Set the field's format and number of components, this is very important!
        entry->format = EXIF_FORMAT_RATIONAL;
        entry->components = 1;

        ExifRational altr;
        altr.numerator = (unsigned)(dalt * 1000000);
        altr.denominator = (unsigned)1000000 ;
        exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, altr);
    }

    /* GPS Lon Ref */
    {
        char value[4] = "West";
        if (dlon > 0) memcpy(value, "East", sizeof(value));

        printf("lon ref %d\n", sizeof(value));
        entry = create_tag(mExifData, EXIF_IFD_GPS, EXIF_TAG_GPS_LONGITUDE_REF, sizeof(value));
        /* Write the actual comment text, without the trailing NUL character */
        memcpy(entry->data, value, sizeof(value));
        /* create_tag() happens to set the format and components correctly for
         * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */
        entry->format = EXIF_FORMAT_ASCII;
    }

    /* GPS Lat Ref */
    {
        char value[5] = "South";
        if (dlat > 0) memcpy(value, "North", sizeof(value));

        printf("lat ref %d\n", sizeof(value));
        entry = create_tag(mExifData, EXIF_IFD_GPS, EXIF_TAG_GPS_LATITUDE_REF, sizeof(value));
        /* Write the actual comment text, without the trailing NUL character */
        memcpy(entry->data, value, sizeof(value));
        /* create_tag() happens to set the format and components correctly for
         * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */
        entry->format = EXIF_FORMAT_ASCII;
    }

    /* GPS Alt Ref */
    {
        // https://github.com/avsej/exif_geo_tag/blob/master/ext/exif_geo_tag.c
        int value = 0;  // above sea

        printf("Alt ref %d\n", sizeof(value));
        entry = create_tag(mExifData, EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE_REF, sizeof(value));
        /* Write the actual comment text, without the trailing NUL character */
        memcpy(entry->data, &value, sizeof(value));
        /* create_tag() happens to set the format and components correctly for
         * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */
        entry->format = EXIF_FORMAT_BYTE;
        entry->components = 1;
    }

    //Write back exif data
    jpeg_data_set_exif_data(mJpegData,mExifData);

    //Save to JPG
    jpeg_data_save_file(mJpegData,"test.jpg");

    return 0;
}
