# Support for NG-ULTRA NX2H540TSC (quad-core ARM Cortex R52) micro-architecture in OTAWA.


## Dependencies
- otawa support for armv7 ISA : [armv7t](https://github.com/statinf-otawa/armv7t)
- otawa loader for arm : [otawa-arm](https://github.com/statinf-otawa/otawa-arm)
- [otawa-clp](https://git.renater.fr/anonscm/git/otawa/otawa-clp.git)
- otawa support for dcache mem : [otawa-dcache](https://github.com/statinf-otawa/otawa-dcache)

## Compilation
    $ cmake . && make install

## Usage
    $ owcet -s Ultra path/to/elf/file functionName [-p CONFIG=x]
    
### Testing

To enable testing,

	$ cmake . -DWITH_TEST=yes
    
To launch a test,

	$ cd test
	$ make

