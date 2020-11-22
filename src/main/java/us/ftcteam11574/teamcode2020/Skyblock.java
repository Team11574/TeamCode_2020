//for speed up
//lower accuracy on orginal pass through
//read all three at same time
//sort using quick alg on topn, then just slice indicies
//IF needed, reduce accuracy on second part
//prfile this, and see hwo long each part takes
//reduced code down to only 360 lines, used to be 516
package us.ftcteam11574.teamcode2020;


import java.util.Arrays;


class Skyblock {
    static final int dwn_sample = 3; //3 probably works best
    static final int a = 30/dwn_sample; //size of read of pixels
    static final int b = 30/dwn_sample; //size of read of pixels
    static final int acc = 6/dwn_sample; //accuracy of read //make acc and acc1/ acc2 go down, //works up to 8 or 9
    //a little less accurate with acc = 2, but its much faster
    static int width;
    static int height;
    static color[] pixels;
    static final int acc1 = 60/dwn_sample;  //different acc etc //works fine at 40, 60 might be a bit large, but we really need speed here
    static final int acc2 = acc1;
    static final int x_size = 200/dwn_sample; //test different sizes etc
    static final int y_size = 120/dwn_sample;

    //VALUES THAT WE TRUNCATE
    static final int y_min = (int) (480 * .23); //this works better when its pretty low on the examples I have //.30 was working, goign bak to taht
    static final int y_max = (int) (480*.50); //This oudl probably be more aggressive
    //will need to truncate on the x side to, since we cna see too many blocks right now


    //NOTE:
        //WIDTH =  640
        //HEIGHT = 480

    //540 by 360
    //Note:
        //Could also add a trunacate for the x part

    static int file_num = 0;


    public static int[] returnCenter(int[][] image,int width,int height) {
        //BufferedImage image = resize(loadImage(loc),1080/dwn_sample,720/dwn_sample);
        //I don't think we have the ability to resize, since buffered image isn't a part of the pacakages provided, will have to find a way to include it
        //If i do want to resize the image


        //readImage(image);
        readImage(image,width,height);
        long time = System.currentTimeMillis();
        double[][] regions = bestRegion(score(pixels));

        double max = 0;
        int max_id = 0;

        for (int i = 0; regions.length > i; i++) {
            if (regions[i][0] > max) {
                max_id = i;
                max = regions[i][0];

            }
        }

        double[] xy= new double[]{regions[max_id][2],regions[max_id][1]};
	   /*
	    for (int i = 0; x_size > i; i++) {
	    	for (int j = 0; y_size > j; j++) {
	    	if (( (i+xy[0] + (x_size/2.)) - sz_draw/2) < width && (int) ( (xy[1] + (j+y_size/2.) ) -sz_draw/2) < height) {
	    		raster.setPixel((int) (xy[0] + i), (int) (xy[1] + j), new int[]{0,0,0});
	    		}
	    	}
	    }
	    */




        return new int[]{(int) (xy[0] + x_size/2),(int) (xy[1] + y_size/2)};
    }



    static void readImage(int[][] rgb, int width_, int height_) {

        int tmp = 0;
        width = width_;
        height =height_;
        color[] pixels = new color[width*height];
        for( int j = 0; j < height; j++ ){
            for( int i = 0; i < width; i++ ) {
                int[] color = rgb[i + j*width];
                pixels[tmp++] = new color(color[0],color[1],color[2]);
            }
        }
        Skyblock.pixels = pixels;
    }





    static double[][] score(color[] pixels) {
        //its possible that I could make pixel info smaller, since many of the values (acc^2 to be exact), are reduant for every pixel
        //
        double[][] pixel_info = new double[pixels.length][3]; //I could make this a float to improve speed, since floats are smaller
        for (int j = 0; height-a > j; j+=acc) {
            for (int i = 0; width-b > i; i+=acc) {
                //System.out.println(("" + (i + (j)*width) + "::::") +  (pixels[i + (j)*width]).red() );

                double[] val ;
                if(j < y_min || j > y_max) {
                    val = new double[]{-100,-100,-100}; //just skip doing the calculations
                    //might need to be more negative, but I think this should be fine
                }
                else {
                    //do the real calcuations
                    val = Skyblock.calcAll(getPixels(pixels,i,j,a,b),a);
                }
                //System.out.println(val[0]);
                for (int l = 0; acc > l; l++) {
                    for (int k = 0; acc > k; k++) {


                        pixel_info[i + (j+l)*width +k] = val;

                    }
                }





            }
        }
        return pixel_info;
    }
    static double[][] bestRegion(double[][] pixel_info) {

        double[][] vals = new double[ (height-y_size)/acc1 * (width-x_size/acc2) ][3];
        int tmp = 0;
        for (int j = 0; height > j; j+=acc1) { //this needs to move in larger chunks
            for (int i = 0; width > i; i+=acc2) {
                tmp++;
                if (j < y_min || j > y_max) {
                    vals[tmp] = new double[] {-100,j,i}; //just socre super badly if not in range
                }
                else{
                    vals[tmp] = new double[] {(double) (Skyblock.combinePixelInfo(pixel_info,i,j,x_size,y_size,new int[]{500/dwn_sample,3000/dwn_sample,500/dwn_sample})),j,i};	    }
            }}
        return vals;
    }
    static double combinePixelInfo(double[][] pass, int xs, int ys, int x, int y,int[] topn) {
        double[][] info = getPixels(pass, xs,ys, x, y);
		  /* Slower method, but more accruate
		  double top0 = skyblock.sum(skyblock.topn(info,0,topn[0])) / Math.pow(topn[0],.85) * 3;
		  double top1 = skyblock.sum(skyblock.topn(info,1,topn[1])) /  Math.pow(topn[1],.85) * 45.5 ;
		  double top2 = skyblock.sum(skyblock.topn(info,2,topn[2])) /  Math.pow(topn[2],.85)  * 20;
		  */
        double top0 = Skyblock.sum(info,0) / Math.pow(topn[0],.85) * 3;
        double top1 = Skyblock.sum(info,1) /  Math.pow(topn[1],.85) * 55.5 ;
        double top2 = Skyblock.sum(info,2) /  Math.pow(topn[2],.85)  * 5;



        return ((top0)+(top1)+(top2));

    }

    static double sum(double[] lst) {
        double sum = 0;
        for (int i = 0; lst.length > i; i++) {
            sum += lst[i];
        }
        return sum;
    }
    static double sum(double[][] lst,int id) {
        double sum = 0;

        for (int i = 0; lst.length > i; i++) {
            sum += lst[i][id];

        }
        return sum;
    }

    static double[] topn(double[][] info, int id, int topn) {
        double[] getId = new double[info.length];
        for (int i = 0; info.length > i; i++) {
            getId[i] = info[i][id];
        }
        double[] res = new double[topn];
        Arrays.sort(getId);
        for (int i = 0; topn > i; i++) {
            //System.out.println(getId.length);
            res[i] = getId[(getId.length-1)-i];
        }
        return res;
    }
    static color[] getPixels(final color[] pxls, final int ws,final int hs,final int w,final int h) {
        final int s = ws + hs*width;
        return getPixels(pxls,s,w,h);
    }
    static color[] getPixels(final color[] pxls,final int s,final int w,final int h) {
        color[] res = new color[w * h];
        for (int i = 0; h > i; i++) {
            final int mult= width*i;
            for (int j = 0; w > j; j++) {

                res[i*w + j] = pxls[s + j + mult];

            }
            //System.out.println(res[i*w]); should always be equal
        }
        return res;
    }
    static double[][] getPixels(double[][] pxls,final  int ws,final int hs,final  int w,final  int h) {
        final int s = ws + hs*width;
        return getPixels(pxls,s,w,h);
    }
    static double[][] getPixels(double[][] pxls, final int s, final int w, final int h) {
        double[][] res = new double[w * h][0];
        for (int i = 0; h > i; i++) {
            boolean will_pass = (w + (s%width) > width) && ((s+w+(h*width))>pxls.length-1);
            if (will_pass) { //this isn't all that elegant, but its faster to split into these two sections
                for (int j = 0; w > j; j++) {

                    if (s + j + (width * i) > pxls.length - 1) {
                        res[i * w + j] = new double[]{0, 0, 0}; //assume matched nothing
                    } else if (j + (s % width) > width) {
                        res[i * w + j] = new double[]{0, 0, 0}; //assume matched nothing if went over on right
                    } else {
                        res[i * w + j] = pxls[s + j + (width * i)];
                    }


                }
            }
            else {
                for (int j = 0; w > j; j++) {
                    res[i * w + j] = pxls[s + j + (width * i)];
                }
            }
            //System.out.println(res[i*w]); should always be equal
        }
        return res;
    }
    static double[] calcAll(color[] pxls,int w) {
        final color black = new color(0,0,0);
        final color yellow = new color(255,170,29); //could be improved
        int[] s1 = new int[2];
        int[] s2 = new int[2];
        int[] s3 = new int[2];
        int[] s4 = new int[2];
        double cur= 0;
        double cur2= 0;
        double res= 0;
        double res2= 0;
        final double cnst = (255+170.0+ (255-29));
        final int div = pxls.length/2;
        for (int i =0; pxls.length > i; i++) { //only looks for horizontal pixels that are the same currently
            final int ylw = Skyblock.colorDist(yellow, pxls[i]);
            final int blc = Skyblock.colorDist(black, pxls[i]);


            double val = (cnst - ylw) /(cnst);
            cur += val;
            cur *= .99;
            res += val * cur*cur*cur;
            //----------
            double val2 = ( cnst  - blc) /(cnst);
            cur2 += val2;
            cur2 *= .99;
            res2 += val2 * cur2*cur2*cur2;
            //----------
            int dist2 = 340-blc;
            int dist1 = 340-ylw;

            if(i % w < w/2) { //first horizontal half
                s1[0] += dist1; //yellow
                s2[0] += dist2; //black
            }
            else { //second horizontal
                s1[1] += dist2; //black
                s2[1] += dist1; //yellow
            }
            if(i < div) { //first vertical half
                s3[0] += dist1; //yellow
                s4[0] += dist2; //black
            }
            else { //second vertical half
                s3[1] += dist2; //black
                s4[1] += dist1; //yellow
            }





        }

        //println(1.3*((double)max(new int[]{score(s1),score(s2),score(s3),score(s4)})) / (255.* pxls.length));
        return new double[] {(3./10000)* (res/pxls.length)/255.,(6./10000)*(res2/pxls.length)/255.,1.3*((double)Skyblock.max(new int[]{Skyblock.score(s1),Skyblock.score(s2),Skyblock.score(s3),Skyblock.score(s4)})) / (255.* pxls.length) };


    }
    static int max(int[] vals) {
        int max = vals[0];
        for (int i = 1; vals.length > i; i++) {
            if (vals[i] > max) {
                max = vals[i];
            }
        }
        return max;
    }
    static int score(int[] s) {
        if(s[0] > s[1]) {
            return (int) ((s[0] + s[1]) * Math.abs(((double)s[1])/s[0]));
        }
        return (int) ((s[0] + s[1]) * Math.abs(((double)s[0])/s[1]));
    }
    static int colorDist(color a, color b) {




        return (int) (Skyblock.abs(a.red()-b.red()) +  Skyblock.abs(a.green()-b.green()) +  Skyblock.abs(a.blue()-b.blue()));
    }
    static int abs(int v) {
        if (v < 0) {
            return -v;
        }
        return v;
    }






}
