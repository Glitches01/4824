/*
    TEST PROGRAM #1: copy memory contents of 16 elements starting at
                     address 0x1000 over to starting address 0x1100.


    long output[16];

    void
    main(void)
    {
      long i;
      *a = 0x1000;
          *b = 0x1100;

      for (i=0; i < 16; i++)
        {
          a[i] = i*10;
          b[i] = a[i];
        }
    }
*/
    data = 0x1000
    li  x1, 1
    add  x2, x1, x0
    add  x3, x2, x1
    add  x4, x3, x2
    add  x5, x4, x3
    add  x6, x5, x4
    add  x7, x6, x5
    add  x8, x7, x6
    add  x9, x8, x7
    li	x2, data
    li  x31, 0x0a
loop:	mul	x3,	x6,	x31
    sw	x3, 0(x2)
    lw	x4, 0(x2)
    sw	x4, 0x100(x2)
    addi	x2,	x2,	0x8 #
    addi	x6,	x6,	0x1 #
    slti	x5,	x6,	16 #
    bne	x5,	x0,	loop #
    wfi
