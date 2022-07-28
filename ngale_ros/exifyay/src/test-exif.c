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
    char mInputFilename[]="example.jpg";

    //Load JPG
    JPEGData * mJpegData = jpeg_data_new_from_file(mInputFilename);

    //Load Exif data from JPG
    ExifData * mExifData = jpeg_data_get_exif_data(mJpegData);

    //Set some Exif options
    exif_data_set_option(mExifData, EXIF_DATA_OPTION_FOLLOW_SPECIFICATION);
    exif_data_set_data_type(mExifData, EXIF_DATA_TYPE_COMPRESSED);
    exif_data_set_byte_order(mExifData, FILE_BYTE_ORDER);

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

    //Write back exif data
    jpeg_data_set_exif_data(mJpegData,mExifData);

    //Save to JPG
    jpeg_data_save_file(mJpegData,"test.jpg");

    return 0;
}
