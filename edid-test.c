#include <stdio.h>

#include "edid.h"

int main(int argc, char *argv[])
{
    struct edid e = {0};

    int res = parse_edid_structure(edid_data, &e);
    if (res != EDID_OK) {
        printf("EDID FAILURE\n");
    }
    else {
        print_edid(&e);
    }

    return 0;
}
