sed -r 's/(#define VERSION )(.)(V0\.)([0-9]+)(.)(.*)/echo "\1\\\"\3$((\4+1))\\\"\6"/ge' include/version.h -i
make clean
if make
then
    v=$(grep VERSION include/version.h | sed -e 's/#define VERSION \"//' -e 's/\"//')
    scp out/firmware/rom0.bin vcspiel:/var/www/csensor/rom0-${v}.bin
    ssh vcspiel "rm /var/www/csensor/rom0-default.bin"
    ssh vcspiel "ln -s /var/www/csensor/rom0-${v}.bin /var/www/csensor/rom0-default.bin"
    echo "Deployed version ${v}."
    ./serialpy.sh  
fi

