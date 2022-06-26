package us.ftcteam11574.teamcode2020;
class color {
    int red;
    int green;
    int blue;
    color(int r, int g, int b) {
        red=r;
        green = g;
        blue = b;
    }

    void setColor(byte[] bytes) {
        //https://github.com/jeremycole/GoldMineralLocator/blob/master/src/us/jcole/opencv/GoldMineralLocator.java

        blue = (int) ((bytes[0])& 0xff);
        green = (int) ((bytes[1])& 0xff);
        red = (int) ((bytes[2])& 0xff);
    }
    static int readBin(byte bin) {
        return readBin(String.format("%8s", Integer.toBinaryString(bin & 0xFF)).replace(' ', '0'));
    }
    static int readBin(String bin) {
        int res = 0;
        int val = 1 << (bin.length()-1);
        for (int i = 0; bin.length() > i; i++) {
            if(bin.charAt(i) == '1') {
                res += val;
            }
            val = (val >> 1);
        }
        return res;
    }
    int red() {
        return red;
    }
    int green() {
        return green;
    }
    int blue() {
        return blue;
    }
    static byte[] getColor(byte[] pixels, int id) {
        return new byte[]{pixels[id*3],pixels[id*3 + 1],pixels[id*3+2]}; //assume no Alpha channel
    }
    static int cast(byte val) {
        return readBin(val);
    }

}