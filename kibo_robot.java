package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.api.types.ImuResult;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.acos;
import static java.lang.Math.asin;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    //variables_declare
    String s = null;

    int ptrn, nrst_mrkr, loop_max;

    double ap_x,ap_y,ap_z,ap_x_copy,ap_y_copy,ap_z_copy,ap_qx,ap_qy,ap_qz,ap_qw,app_qx,app_qy,app_qz,app_qw,f_qw,f_qx,f_qy,f_qz;

    double x_max = 11.55,z_max = 5.57,x_min = 10.3,z_min = 4.32,min_dstc = 0.165;

    double x_max_tlrnc=x_max-min_dstc;
    //x_max_tlrnc  11.385
    double z_max_tlrnc=z_max-min_dstc;
    //z_max_tlrnc  5.405
    double x_min_tlrnc=x_min+min_dstc;
    //x_min_trlnc  10.465
    double z_min_tlrnc=z_min+min_dstc;
    //z_min_trlnc  4.485

    double [][] transVectors = new double[4][3];

    double target_x_dstc,target_y_dstc,target_z_dstc;
    double target_to_marker_x,target_to_marker_y=0,target_to_marker_z;

    double[][] ptrn_7_break_points = new double[2][3];

    Bitmap bmp = null;

    double theta, laser_final_x, laser_final_y, laser_final_z;

    double [] rotation_axis = new double[3];
    double [] second_rotation_axis = new double[3];



    @Override
    protected void runPlan1(){

        // write here your plan 1

        //point_B_path_plan_shorten
        //ar_detect_image_crop
        //laser_accuracy_4_marker_data_filter
        //laser_accuracy_image_processing
        //laser_accuracy_laser_distance_from_center
        //path_plan_according_to_kinematic_data


        api.startMission();
        Log.i("L_KITT", "_____________mission_started________________");
        curr_status();

        //intermediate_point
        Log.i("L_KITT", "moving_to_intermediate_point__10.2,-9.8,4.6,0,0,0,1");
        loop_max = 0;
        move(10.2,-9.8,4.6,0,0,0,1);
        Log.i("L_KITT", "move_func_complete");
        curr_status();

        //point_A
        Log.i("L_KITT", "moving_to_point_A__11.21,-9.8,4.79,0,0.0,-0.707,0.707");
        loop_max = 3;
        move(11.21,-9.8,4.79,0,0.0,-0.707,0.707);
        Log.i("L_KITT", "move_func_complete");
        curr_status();

        //QR_scan
        Log.i("L_KITT", "qr_scan_func_starting");
        sendQR();
        Log.i("L_KITT", "QR_scan_func_completed");
        if(s == null){
            Log.i("L_KITT", "qr_scan_func_starting_2nd_time");
            sendQR();
        }
        Log.i("L_KITT", s);

        //making_substring_of_discovered_data
        decodeQR();
        Log.i("L_KITT", "QR_decode_complete");
        Log.i("L_KITT","pattern="+ptrn+"___co-ordinates_A'(x)="+ap_x+"___A'(y)="+ap_y+"___A'(z)="+ap_z);

        //changing_coordinate_of_A'_to_avoid_KOZ
        Log.i("L_KITT", "KOZ_factor_calculation_starting");
        avdKOZ();
        Log.i("L_KITT", "KOZ_factor_calculation_complete");
        Log.i("L_KITT","pattern="+ptrn+"___co-ordinates_A'(x)="+ap_x+"___A'(y)="+ap_y+"___A'(z)="+ap_z);

        //path_plan_to_move_A'
        if(ptrn != 2) {
            Log.i("L_KITT", "path_plan_execution_starting");
            more_move2();
            Log.i("L_KITT", "path_plan_executed");
        }

        if(ptrn != 7) {
            //move_to_point_A'
            Log.i("L_KITT", "moving_to_A'___" + ap_x + "," + ap_y + "," + ap_z + "0,0,-0.707,0.707");
            loop_max = 3;
            move(ap_x, ap_y, ap_z, 0, 0, -0.707, 0.707);
            Log.i("L_KITT", "move_func_complete");
            curr_status();
        }
        //ar_detection_function
        Log.i("L_KITT", "entering_detectAR_function");
        detectAR();
        Log.i("L_KITT", "detectAR_function_complete");

        //calculate_quaternion_to_change
        Log.i("L_KITT","calculating_quaternion_to_focus_target");
        targetQua();
        Log.i("L_KITT","calculation_complete");

        //changing_quaternion_to_focus_on_target
        Log.i("L_KITT","orientation_changing_to_be_on_line_of_target");
        Log.i("L_KITT", "moving_to_____" + ap_x + "__" + ap_y + "__" + ap_z);
        loop_max = 3;
        //relative_move(0.0,0.0,0.0,ap_qx,ap_qy,ap_qz,ap_qw);
        move(ap_x,ap_y,ap_z,f_qx,f_qy,f_qz,f_qw);
        Log.i("L_KITT","move_func_complete");
        curr_status();

        //laser_on
        Log.i("L_KITT","turing_laser_on");
        api.laserControl(true);

        //taking_snapshots
        Log.i("L_KITT","taking_snapshot");
        api.takeSnapshot();
        Log.i("L_KITT","snapshot_taken");

        //laser_off
        api.laserControl(false);
        Log.i("L_KITT","Laser_turned_off");

        //path_plan_and_excute_to_point_B
        Log.i("L_KITT","path_planning_to_go_B");
        gotoB();
        Log.i("L_KITT","gotoB_func_complete");

        Log.i("L_KITT","________________mission_complete____________________");
        api.reportMissionCompletion();


    }

    @Override
    protected void runPlan2(){
// write here your plan 2
    }

    @Override
    protected void runPlan3(){
// write here your plan 3
    }


    private void move(double pos_x, double pos_y, double pos_z, double qua_x, double qua_y, double qua_z, double qua_w) {

        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y, (float)qua_z, (float)qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;

        while(!result.hasSucceeded() || loopCounter < loop_max){
            Log.i("L_KITT", "move_func_loop_count=" + loopCounter);
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
            if(loopCounter > 5){
                Log.i("L_KITT", "move_func_exec_failed");
                break;
            }
        }

    }

    private void relative_move(double post_x, double post_y, double post_z, double quat_x, double quat_y, double quat_z, double quat_w) {

        final Point point = new Point(post_x, post_y, post_z);
        final Quaternion quaternion = new Quaternion((float)quat_x, (float)quat_y, (float)quat_z, (float)quat_w);

        Result result = api.relativeMoveTo(point, quaternion, true);

        int loopCounter = 0;

        while(!result.hasSucceeded() || loopCounter < loop_max){
            Log.i("L_KITT", "relative_move_func_loop_count=" + loopCounter);
            result = api.relativeMoveTo(point, quaternion, true);
            ++loopCounter;
            if(loopCounter > 5){
                Log.i("L_KITT", "relative_move_func_exec_failed");
                break;
            }
        }

    }


    private void btmp_Navcam() {

        //capture_image
        bmp = null;
        Log.i("L_KITT", "capturing_image");
        bmp = api.getBitmapNavCam();
        Log.i("L_KITT", "image_captured");

        int loopCounter = 0;

        //if_err_occurs_recapture
        while(bmp == null && loopCounter<loop_max){
            bmp = api.getBitmapNavCam();
            loopCounter++;
            Log.i("L_KITT","loop_count = " + loopCounter);
        }
        if(loopCounter==3){
            Log.i("L_KITT","err_capturing_image");
        }


        Log.i("L_KITT","height_____"+bmp.getHeight());
        Log.i("L_KITT","width______"+bmp.getWidth());


    }

    private void sendQR() {

        loop_max = 3;
        //flash_on
        api.flashlightControlFront((float)0.8);
        Log.i("L_KITT", "flash_on");

        btmp_Navcam();

        //flash_off
        api.flashlightControlFront(0);
        Log.i("L_KITT", "flash_off");

        //crop_image
        bmp = Bitmap.createBitmap(bmp, 600 , 600, 250,250);

        Log.i("L_KITT","cropped_height_____"+bmp.getHeight());
        Log.i("L_KITT","cropped_width______"+bmp.getWidth());

        int[] binaryImage = new int[bmp.getHeight() * bmp.getWidth()];
        bmp.getPixels(binaryImage, 0, bmp.getWidth(), 0, 0, bmp.getWidth(), bmp.getHeight());
        LuminanceSource ls = new RGBLuminanceSource(bmp.getWidth(), bmp.getHeight(), binaryImage);
        com.google.zxing.Result r = null;
        try {
            r = new MultiFormatReader().decode(new BinaryBitmap(new HybridBinarizer(ls)));
        } catch (Exception e) {
            e.printStackTrace();
        }
        if (r != null) {
            api.sendDiscoveredQR(r.getText());
            s = r.getText();
            Log.i("L_KITT","QR_discovered_and_sent");
        }
    }

    private void decodeQR() {
        s = s.substring(1,s.length()-1);
        String [] strs = s.split(",");
        ptrn = Integer.parseInt(strs[0].substring(4));
        ap_x = Double.parseDouble(strs[1].substring(4));
        ap_x_copy=ap_x;
        ap_y = Double.parseDouble(strs[2].substring(4));
        ap_y_copy=ap_y;
        ap_z = Double.parseDouble(strs[3].substring(4));
        ap_z_copy=ap_z;
    }

    private void avdKOZ() {

        //to_avoid_collision
        //root2 * (0.08+koz_fctr) = root3 * 0.16
        double koz_fctr=0.12;
        double koz_fctr_corn = 0.2;

        //pattern_1___x+___z-
        //pattern_7___x+___z+
        if(ptrn==1 || ptrn==7){
            if(ap_x+koz_fctr<=x_max_tlrnc){
                ap_x += koz_fctr;
            }
            else{
                ap_x=x_max_tlrnc;
            }
            if(ptrn==1){
                if(ap_z-koz_fctr>=z_min_tlrnc){
                    ap_z -= koz_fctr;
                }
                else{
                    ap_z=z_min_tlrnc;
                }
            }
            else {
                if(ap_z+koz_fctr<=z_max_tlrnc){
                    ap_z += koz_fctr;
                }
                else{
                    ap_z= z_max_tlrnc;
                }
            }
        }

        //pattern_3___x-___z-
        //pattern_5___x-___z+
        else if(ptrn==3 ||ptrn==5){
            if(ap_x-koz_fctr>= x_min_tlrnc) {
                ap_x -= koz_fctr;
            }
            else{
                ap_x=x_min_tlrnc;
            }
            if(ptrn==3){
                if(ap_z-koz_fctr>=z_min_tlrnc) {
                    ap_z -= koz_fctr;
                }
                else{
                    ap_z=z_min_tlrnc;
                }
            }
            else {
                if(ap_z+koz_fctr<=z_max_tlrnc){
                    ap_z += koz_fctr;
                }
                else{
                    ap_z=z_max_tlrnc;
                }
            }
        }

        //pattern_2___corner_z-
        else if(ptrn==2){
            if(ap_z-koz_fctr_corn>=z_min_tlrnc) {
                ap_z -= koz_fctr_corn;
            }
            else{
                ap_z=z_min_tlrnc;
            }
        }

        //pattern_4___corner_x-
        else if(ptrn==4){
            if(ap_x-koz_fctr_corn>=x_min_tlrnc) {
                ap_x -= koz_fctr_corn;
            }
            else{
                ap_x=x_min_tlrnc;
            }
        }

        //pattern_6___corner_z+
        else if(ptrn==6){
            if(ap_z+koz_fctr_corn<=z_max_tlrnc){
                ap_z += koz_fctr_corn;
            }
            else{
                ap_z=z_max_tlrnc;
            }
        }

        //pattern_8___corner_x+
        else if(ptrn==8){
            if(ap_x+koz_fctr_corn<=x_max_tlrnc) {
                ap_x += koz_fctr_corn;
            }
            else{
                ap_x=x_max_tlrnc;
            }
        }
    }

    /*
    private void more_move() {
        double r_x,r_y,r_z;
        r_y=ap_y;
        r_z=4.79;
        double dis_fctr=0.295;

        if(ptrn==3 || ptrn==4){
            r_x=ap_x;
            Log.i("L_KITT","moving_to_____"+r_x+"__"+r_y+"__"+r_z);
            loop_max = 0;
            move(r_x,r_y,r_z,0,0,-0.707,0.707);
            Log.i("L_KITT","move_func_complete");
            curr_status();
        }
        else if(ptrn==5 || ptrn==6){
            if(ap_x-dis_fctr> x_min_tlrnc) {
                r_x = ap_x - dis_fctr;
            }
            else{
                r_x=x_min_tlrnc;
            }
            if (ptrn == 6) {
                if(r_x-0.24> x_min_tlrnc) {
                    r_x -= 0.24;
                }
                else{
                    r_x=x_min_tlrnc;
                }
            }
            Log.i("L_KITT","moving_to_____"+r_x+"__"+r_y+"__"+r_z);
            loop_max = 0;
            move(r_x,r_y,r_z,0,0,-0.707,0.707);
            Log.i("L_KITT","move_func_complete");
            curr_status();

            r_z=ap_z;
            Log.i("L_KITT","moving_to_____"+r_x+"__"+r_y+"__"+r_z);
            loop_max = 0;
            move(r_x,r_y,r_z,0,0,-0.707,0.707);
            Log.i("L_KITT","move_func_complete");
            curr_status();
        }
        else if(ptrn==7){
            if(ap_x+dis_fctr<x_max_tlrnc) {
                r_x = ap_x + dis_fctr;
            }
            else{
                r_x=x_max_tlrnc;
            }
            Log.i("L_KITT","moving_to_____"+r_x+"__"+r_y+"__"+r_z);
            loop_max = 0;
            move(r_x,r_y,r_z,0,0,-0.707,0.707);
            ptrn_7_break_points[0][0] = r_x;
            ptrn_7_break_points[0][1] = r_y;
            ptrn_7_break_points[0][2] = r_z;
            Log.i("L_KITT","move_func_complete");
            curr_status();

            r_z=ap_z;
            Log.i("L_KITT","moving_to_____"+r_x+"__"+r_y+"__"+r_z);
            loop_max = 0;
            move(r_x,r_y,r_z,0,0,-0.707,0.707);
            ptrn_7_break_points[1][0] = r_x;
            ptrn_7_break_points[1][1] = r_y;
            ptrn_7_break_points[1][2] = r_z;
            Log.i("L_KITT","move_func_complete");
            curr_status();
        }
    }
    */

    private void more_move2() {

        double mid_x, mid_z, koz_z_min_tlrc=5.015;


        if(ptrn == 1){
            //to determine the position of koz in path colliding or not
            //area of 3 points in XZ plane
            //A',   (mid_x,mid_z)=((koz_position-a')both respect to laser+min_dstc(to avoid collision)+ actual a', (koz_position-a')respect to laser+min_dstc+ actual a'),   A(11.21,4.79)

            mid_x = -0.075-0.08+min_dstc+ap_x_copy;
            mid_z = -0.3+0.08-min_dstc+ap_z_copy;

            if(mid_x>x_max_tlrnc){
                mid_x=x_max_tlrnc;
            }
            if(mid_z<(koz_z_min_tlrc-min_dstc)){
                mid_z = koz_z_min_tlrc-min_dstc;
            }

            double [][] triangle= {{11.21,4.79},{ap_x,ap_z},{mid_x,mid_z}};
            if(area_estimation(triangle) < 0){
                Log.i("L_KITT","moving_to_____"+mid_x+"__"+ap_y+"__"+mid_z);
                loop_max=0;
                move(mid_x,ap_y,mid_z,0,0,-0.707,0.707);
                Log.i("L_KITT","move_func_complete");
                curr_status();
            }
        }
        else if(ptrn == 3){

            //(0.075+0.08+A'x-dis_fact , -0.3+0.08+A'z-dis_fact) == (midx,midy)

            mid_x = 0.075+0.08-min_dstc+ap_x_copy;
            mid_z = -0.3+0.08-min_dstc+ap_z_copy;

            if(mid_x<x_min_tlrnc){
                mid_x=x_min_tlrnc;
            }
            if(mid_z<(koz_z_min_tlrc-min_dstc)){
                mid_z = koz_z_min_tlrc-min_dstc;
            }

            double [][] triangle= {{11.21,4.79},{mid_x,mid_z},{ap_x,ap_z}};

            if(area_estimation(triangle) < 0) {
                Log.i("L_KITT", "moving_to_____" + mid_x + "__" + ap_y + "__" + mid_z);
                loop_max = 0;
                move(mid_x, ap_y, mid_z, 0, 0, -0.707, 0.707);
                Log.i("L_KITT", "move_func_complete");
                curr_status();
            }

        }
        else if(ptrn == 4){

            //(0.28+A'x-dist_fact , -0.3+0.08+A'z-dist_fact) == (midx,midy)

            mid_x = 0.08-min_dstc+ap_x_copy;
            mid_z = -0.3+0.08-min_dstc+ap_z_copy;

            if(mid_x<x_min_tlrnc){
                mid_x=x_min_tlrnc;
            }
            if(mid_z<(koz_z_min_tlrc-min_dstc)){
                mid_z = koz_z_min_tlrc-min_dstc;
            }

            double [][] triangle= {{11.21,4.79},{mid_x,mid_z},{ap_x,ap_z}};

            if(area_estimation(triangle) < 0) {
                Log.i("L_KITT", "moving_to_____" + mid_x + "__" + ap_y + "__" + mid_z);
                loop_max = 0;
                move(mid_x, ap_y, mid_z, 0, 0, -0.707, 0.707);
                Log.i("L_KITT", "move_func_complete");
                curr_status();
            }

        }

        else if(ptrn==5){

            //(-0.3+0.08+A'xcopy-dist_fact , -0.3-0.08+A'zcopy-dist_fact)

            mid_x = -0.3+0.08+ap_x_copy-min_dstc;
            mid_z = -0.3-0.08+ap_z_copy-min_dstc;
            mid_z-=0.08;

            if(mid_x<x_min_tlrnc){
                mid_x=x_min_tlrnc;
            }
            if(mid_z<(koz_z_min_tlrc-min_dstc)){
                mid_z = koz_z_min_tlrc-min_dstc;
            }



            Log.i("L_KITT", "moving_to_____" + mid_x + "__" + ap_y + "__" + mid_z);
            loop_max = 0;
            move(mid_x, ap_y, mid_z, 0, 0, -0.707, 0.707);
            Log.i("L_KITT", "move_func_complete");
            curr_status();

            //(-0.3+0.08+A'xcopy-dist_fact , -0.075-0.08+A'zcopy+dist_fact)

            //mid_x = -0.3+0.08+ap_x_copy-min_dstc;
            mid_z = -0.075-0.08+ap_z_copy+min_dstc;

            if(mid_z>z_max_tlrnc){
                mid_z=z_max_tlrnc;
            }

            Log.i("L_KITT", "moving_to_____" + mid_x + "__" + ap_y + "__" + mid_z);
            loop_max = 0;
            move(mid_x, ap_y, mid_z, 0, 0, -0.707, 0.707);
            Log.i("L_KITT", "move_func_complete");
            curr_status();


        }

        else if(ptrn==6){

            //(-0.3+0.08+A'xcopy-dist_fact , -0.3-0.08+A'zcopy-dist_fact )

            mid_x = -0.3+0.08+ap_x_copy-min_dstc;
            mid_z = -0.3-0.08+ap_z_copy-min_dstc;
            mid_z-=0.08;

            if(mid_x<x_min_tlrnc){
                mid_x=x_min_tlrnc;
            }
            if(mid_z<(koz_z_min_tlrc-min_dstc)){
                mid_z = koz_z_min_tlrc-min_dstc;
            }


            Log.i("L_KITT", "moving_to_____" + mid_x + "__" + ap_y + "__" + mid_z);
            loop_max = 0;
            move(mid_x, ap_y, mid_z, 0, 0, -0.707, 0.707);
            Log.i("L_KITT", "move_func_complete");
            curr_status();

            //(-0.3+0.08+A'xcopy-dist_fact , -0.08+A'zcopy+dist_fact )

            //mid_x = -0.3+0.08+ap_x_copy-min_dstc;
            mid_z = -0.08+ap_z_copy+min_dstc;

            if(mid_z>z_max_tlrnc){
                mid_z=z_max_tlrnc;
            }

            Log.i("L_KITT", "moving_to_____" + mid_x + "__" + ap_y + "__" + mid_z);
            loop_max = 0;
            move(mid_x, ap_y, mid_z, 0, 0, -0.707, 0.707);
            Log.i("L_KITT", "move_func_complete");
            curr_status();


        }

        else if(ptrn == 7){

            //(A'xcopy , -0.3-0.08+A'zcopy-(1.73)dis_fact)

            mid_x=ap_x_copy;
            mid_z=-0.3-0.08+ap_z_copy-0.28;
            mid_z-=0.08;

            if(mid_x>x_max_tlrnc){
                mid_x=x_max_tlrnc;
            }
            if(mid_z<(koz_z_min_tlrc-0.28)){
                mid_z = koz_z_min_tlrc-0.28;
            }

            Log.i("L_KITT", "moving_to_____" + mid_x + "__" + ap_y + "__" + mid_z);
            loop_max = 3;
            move(mid_x, ap_y, mid_z, 0, 0, -0.707, 0.707);
            Log.i("L_KITT", "move_func_complete");
            curr_status();
        }

        else if(ptrn == 8){

            //(-0.08+A'xcopy+dist_fact , -0.3+0.08+A'zcopy-dist_fact)

            mid_x = -0.08+min_dstc+ap_x_copy;
            mid_z = -0.3+0.08-min_dstc+ap_z_copy;

            if(mid_x>x_max_tlrnc){
                mid_x=x_max_tlrnc;
            }
            if(mid_z<(koz_z_min_tlrc-min_dstc)){
                mid_z = koz_z_min_tlrc-min_dstc;
            }

            double [][] triangle= {{11.21,4.79},{ap_x,ap_z},{mid_x,mid_z}};
            if(area_estimation(triangle) < 0){
                Log.i("L_KITT","moving_to_____"+mid_x+"__"+ap_y+"__"+mid_z);
                loop_max=0;
                move(mid_x,ap_y,mid_z,0,0,-0.707,0.707);
                Log.i("L_KITT","move_func_complete");
                curr_status();
            }
        }


    }

    private double area_estimation(double[][] p){
        double area;
        area = 0.5*(p[0][0]*(p[1][1]-p[2][1])+p[1][0]*(p[2][1]-p[0][1])+p[2][0]*(p[0][1]-p[1][1]));
        Log.i("L_KITT","area is"+area);
        return area;
    }

    private void detectAR() {

        loop_max = 3;
        btmp_Navcam();

        //set the point
        ap_x=api.getRobotKinematics().getPosition().getX();
        ap_y=api.getRobotKinematics().getPosition().getY();
        ap_z=api.getRobotKinematics().getPosition().getZ();


        //converting_to_mat_image
        Mat img = new Mat(bmp.getWidth(),bmp.getHeight(), CvType.CV_8UC1);
        Utils.bitmapToMat(bmp,img);
        Log.i("L_KITT","image_converted_to_mat");
        Imgproc.cvtColor(img,img, Imgproc.COLOR_BGRA2GRAY);
        Log.i("L_KITT","image_converted_to_gray");

        //inputs_declare_and_define
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        double [][] camera_param = api.getNavCamIntrinsics();
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Mat cameraMatrix = new Mat(3,3,CvType.CV_64F);
        Mat distCoEff = new Mat(5,1,CvType.CV_64F);

        //get_camera_matrix_and_distortion_co-efficient_of_Nav_Cam
        cameraMatrix.put(0,0,camera_param[0]);
        distCoEff.put(0,0,camera_param[1]);
        Log.i("L_KITT","got_camera_matrix_and_distortion_co-efficient_of_Nav_Cam");

        //outputs_declare_and_define
        Mat rvecs = new Mat();
        Mat tvecs = new Mat();

        //marker_ids_discover
        Log.i("L_KITT","marker_ids_discovering");
        Aruco.detectMarkers(img,dictionary,corners,ids);
        for(int i = 0; i < ids.cols(); i++){
            for(int j = 0; j < ids.rows(); j++){
                for(int k = 0; k < ids.channels(); k++){
                    Log.i("L_KITT","marker_ids_["+j+"]["+i+"]["+k+"]___"+ids.get(j,i)[k]);
                }
            }
        }
        //marker_position_determination
        Log.i("L_KITT","marker_position_estimation_starting");
        Aruco.estimatePoseSingleMarkers(corners, (float)0.05, cameraMatrix, distCoEff, rvecs, tvecs);

        for(int i = 0; i < 4; i++){
            int n = (int) ids.get(i,0)[0];
            transVectors[n-1][0] = tvecs.get(i,0)[0];
            transVectors[n-1][1] = tvecs.get(i,0)[1];
            transVectors[n-1][2] = tvecs.get(i,0)[2];
        }

        for(int i = 0; i < 4; i++){
            Log.i("L_KITT","marker___"+(i+1)+"distances");
            for(int j = 0; j < 3; j++){
                Log.i("L_KITT","transvectors_["+i+"]["+j+"]___"+transVectors[i][j]);
            }
        }
    }

    private void targetQua() {

        //nearest_marker_and_target_distance_calculation

        if(ptrn == 1 || ptrn == 8) {
            Log.i("L_KITT","nearest_marker_id___1");
            nrst_mrkr=1;
            target_dstc();
        }

        else if(ptrn == 2 || ptrn == 3 || ptrn == 4) {
            Log.i("L_KITT","nearest_marker_id___2");
            nrst_mrkr=2;
            target_dstc();
        }

        else if(ptrn == 5 || ptrn == 6) {
            Log.i("L_KITT","nearest_marker_id___3");
            nrst_mrkr=3;
            target_dstc();
        }


        else if(ptrn == 7) {
            Log.i("L_KITT","nearest_marker_id___1");
            nrst_mrkr=1;
            target_dstc();
        }
        first_rotation_vector();
        first_rotation_angle();
        //second_rotation();
        second_rotation2();
        final_quaternion();
    }

    private void target_dstc() {

        if(nrst_mrkr==1 || nrst_mrkr==4){
            target_to_marker_x=0.1125;
        }
        else{
            target_to_marker_x=-0.1125;
        }

        if(nrst_mrkr==1 || nrst_mrkr==2){
            target_to_marker_z=-0.0415;
        }
        else{
            target_to_marker_z=+0.0415;
        }

        //distance_respect_to_camera
        target_x_dstc=-target_to_marker_x+transVectors[nrst_mrkr-1][0];
        target_y_dstc=-target_to_marker_y-transVectors[nrst_mrkr-1][2];
        target_z_dstc=-target_to_marker_z+transVectors[nrst_mrkr-1][1];

        Log.i("L_KITT","distance_of_target_from_camera__x="+target_x_dstc+"__y="+target_y_dstc+"__z="+target_z_dstc);

        //distance respect to center of bee
        target_x_dstc-=0.0422;
        target_y_dstc-=0.1177;
        target_z_dstc-=0.0826;

        Log.i("L_KITT","distance_of_target_from_center__x="+target_x_dstc+"__y="+target_y_dstc+"__z="+target_z_dstc);


    }

    private void first_rotation_vector() {

        double scalar_rotation_vec= sqrt(pow(target_y_dstc,2)+pow(target_z_dstc,2));
        ap_qx=0;
        ap_qy=-(target_z_dstc/scalar_rotation_vec);
        ap_qz=(target_y_dstc/scalar_rotation_vec);
        Log.i("L_KITT","rotation_vector="+ap_qx+"i+"+ap_qy+"j+"+ap_qz+"k");

        rotation_axis[0] = ap_qx;
        rotation_axis[1] = ap_qy;
        rotation_axis[2] = ap_qz;

    }

    private void first_rotation_angle() {

        theta = acos(target_x_dstc/(sqrt(pow(target_x_dstc,2)+pow(target_y_dstc,2)+pow(target_z_dstc,2))));
        Log.i("L_KITT","rotation_angle="+theta);
        double theta_p=theta/2;
        ap_qw=cos(theta_p);
        ap_qx=ap_qx*sin(theta_p);
        ap_qy=ap_qy*sin(theta_p);
        ap_qz=ap_qz*sin(theta_p);
        Log.i("L_KITT","quaternion_of_first_rotation___qua_x="+ap_qx+"_qua_y="+ap_qy+"_qua_z="+ap_qz+"_qua_w="+ap_qw);

    }

    private void gotoB() {

        double koz_y_trlnc = -8.8-min_dstc;
        double mid_z = (ap_z+4.5)/2;

        if(ptrn > 1 && ptrn <= 7){
            //shortest_way_hasn't_applied
            Log.i("L_KITT","moving_to_" + x_min_tlrnc+","+ koz_y_trlnc +","+ap_z+",0,0,-0.707,0.707");
            loop_max = 0;
            move(x_min_tlrnc,koz_y_trlnc,ap_z,0,0,-0.707,0.707);
            Log.i("L_KITT","move_func_complete");
            curr_status();
        }
        else if(ptrn == 1 || ptrn == 8){
            Log.i("L_KITT","moving_to_"+ap_x+","+ ap_y +",4.5,0,0,-0.707,0.707");
            loop_max = 0;
            move(ap_x,ap_y,4.5,0,0,-0.707,0.707);
            Log.i("L_KITT","move_func_complete");
            curr_status();

            Log.i("L_KITT","moving_to_"+x_min_tlrnc+","+koz_y_trlnc+",4.5,0,0,-0.707,0.707");
            loop_max = 0;
            move(x_min_tlrnc,koz_y_trlnc,4.5,0,0,-0.707,0.707);
            Log.i("L_KITT","move_func_complete");
            curr_status();
        }
        /*else if(ptrn == 7){
            Log.i("L_KITT","moving_to_"+ptrn_7_break_points[1][0]+","+ptrn_7_break_points[1][1] +","+ptrn_7_break_points[1][2]+",0,0,-0.707,0.707");
            loop_max = 0;
            move(ptrn_7_break_points[1][0],ptrn_7_break_points[1][1],ptrn_7_break_points[1][2],0,0,-0.707,0.707);
            Log.i("L_KITT","move_func_complete");
            curr_status();

            Log.i("L_KITT","moving_to_" +ptrn_7_break_points[0][0]+","+ptrn_7_break_points[0][1] +","+ptrn_7_break_points[0][2]+",0,0,-0.707,0.707");
            loop_max = 0;
            move(ptrn_7_break_points[0][0],ptrn_7_break_points[0][1],ptrn_7_break_points[0][2],0,0,-0.707,0.707);
            Log.i("L_KITT","move_func_complete");
            curr_status();

            Log.i("L_KITT","moving_to_" + x_min_tlrnc+","+ koz_y_trlnc +","+mid_z+",0,0,-0.707,0.707");
            loop_max = 0;
            move(x_min_tlrnc,koz_y_trlnc,mid_z,0,0,-0.707,0.707);
            Log.i("L_KITT","move_func_complete");
            curr_status();
        }*/
        Log.i("L_KITT","moving_to___10.6,-8.0,4.5,0,0,-0.707,0.707");
        loop_max = 0;
        move(10.6,-8.0,4.5,0,0,-0.707,0.707);
        Log.i("L_KITT","move_func_complete");
        curr_status();

    }

    /*private void second_rotation(){
        //get the distance from center of gravity of astrobee to target point = distance_center_target
        //calculate the angle between cog-laser vector and laser direction(after rotation) = phi
        //calculate the distance from laser to cog = b
        //sin(angle_of_rotation) = {b sin (phi)} / distance_center_target -----

        //determination of unit vector
        //cross product of laser direction vector after rotation and cog to target vector

        //determination of laser direction after rotation
        //rodrigues rotation formula

        //quaternion -- angle of rotation and unit vector

        Log.i("L_KITT","second_rotation_calculation_starting");

        rodrigues_formula();

        //sin(theta)=sin(180-theta)
        double sin_phi = 0.692440103;
        double b = 0.180463542;
        double second_rotation_angle = b/(sqrt(pow(target_x_dstc,2)+pow(target_y_dstc,2)+pow(target_z_dstc,2)));
        second_rotation_angle=second_rotation_angle*sin_phi;
        second_rotation_angle = asin(second_rotation_angle);

        Log.i("L_KITT","second_rotation_angle="+second_rotation_angle);

        //for_quaternion
        second_rotation_angle = second_rotation_angle/2;

        second_rotation_axis[0] = laser_final_y*target_z_dstc - target_y_dstc*laser_final_z;
        second_rotation_axis[1] = laser_final_z*target_x_dstc - target_z_dstc*laser_final_x;
        second_rotation_axis[2] = laser_final_x*target_y_dstc - target_x_dstc*laser_final_y;

        double scalar_second_rotation = sqrt(pow(second_rotation_axis[0],2)+pow(second_rotation_axis[1],2)+pow(second_rotation_axis[2],2));

        second_rotation_axis[0]=second_rotation_axis[0]/scalar_second_rotation;
        second_rotation_axis[1]=second_rotation_axis[1]/scalar_second_rotation;
        second_rotation_axis[2]=second_rotation_axis[2]/scalar_second_rotation;

        app_qw = cos(second_rotation_angle);
        app_qx = sin(second_rotation_angle)*second_rotation_axis[0];
        app_qy = sin(second_rotation_angle)*second_rotation_axis[1];
        app_qz = sin(second_rotation_angle)*second_rotation_axis[2];

        Log.i("L_KITT","quaternion_of_second_rotation___qua_x="+app_qx+"_qua_y="+app_qy+"_qua_z="+app_qz+"_qua_w="+app_qw);

    }
    private void rodrigues_formula(){

        //rotation angle theta
        //rotaion axis []
        //calculating_vector_initial_pos
        double ini_x=0.1302 ,ini_y=0.0572, ini_z=-0.1111;
        double dot_product = ini_x*rotation_axis[0]+ini_y*rotation_axis[1]+ini_z*rotation_axis[2];
        laser_final_x = ini_x*cos(theta)+(rotation_axis[1]*ini_z-ini_y*rotation_axis[2])*sin(theta)+dot_product*rotation_axis[0]*(1-cos(theta));
        laser_final_y = ini_y*cos(theta)+(rotation_axis[2]*ini_x-ini_z*rotation_axis[0])*sin(theta)+dot_product*rotation_axis[1]*(1-cos(theta));
        laser_final_z = ini_z*cos(theta)+(rotation_axis[0]*ini_y-ini_x*rotation_axis[1])*sin(theta)+dot_product*rotation_axis[2]*(1-cos(theta));


    }*/

    private void second_rotation2(){

        Log.i("L_KITT","second_rotation_calculation_starting");

        //sin(theta)=sin(180-theta)
        double sin_phi = 0.692440103;
        double b = 0.180463542;
        double second_rotation_angle = b/(sqrt(pow(target_x_dstc,2)+pow(target_y_dstc,2)+pow(target_z_dstc,2)));
        second_rotation_angle=second_rotation_angle*sin_phi;
        second_rotation_angle = asin(second_rotation_angle);

        Log.i("L_KITT","second_rotation_angle="+second_rotation_angle);

        //for_quaternion
        second_rotation_angle = second_rotation_angle/2;

        second_rotation_axis[0] = 0;
        second_rotation_axis[1] = -0.1111;
        second_rotation_axis[2] = -0.0572;

        double scalar_second_rotation = sqrt(pow(second_rotation_axis[0],2)+pow(second_rotation_axis[1],2)+pow(second_rotation_axis[2],2));

        second_rotation_axis[0]=second_rotation_axis[0]/scalar_second_rotation;
        second_rotation_axis[1]=second_rotation_axis[1]/scalar_second_rotation;
        second_rotation_axis[2]=second_rotation_axis[2]/scalar_second_rotation;

        Log.i("L_KITT","second rotation_vector="+second_rotation_axis[0]+"i+"+second_rotation_axis[1]+"j+"+second_rotation_axis[2]+"k");

        app_qw = cos(second_rotation_angle);
        app_qx = sin(second_rotation_angle)*second_rotation_axis[0];
        app_qy = sin(second_rotation_angle)*second_rotation_axis[1];
        app_qz = sin(second_rotation_angle)*second_rotation_axis[2];

        Log.i("L_KITT","quaternion_of_second_rotation___qua_x="+app_qx+"_qua_y="+app_qy+"_qua_z="+app_qz+"_qua_w="+app_qw);

    }

    private void final_quaternion(){
        f_qw = ap_qw*app_qw -ap_qx*app_qx -ap_qy*app_qy -ap_qz*app_qz;
        f_qx = ap_qw*app_qx +ap_qx*app_qw +ap_qy*app_qz -ap_qz*app_qy;
        f_qy = ap_qw*app_qy -ap_qx*app_qz +ap_qy*app_qw +ap_qz*app_qx;
        f_qz = ap_qw*app_qz +ap_qx*app_qy -ap_qy*app_qx +ap_qz*app_qw;

        Log.i("L_KITT","quaternion_to_move_target___qua_x="+f_qx+"_qua_y="+f_qy+"_qua_z"+f_qz+"_qua_w="+f_qw);
    }

    private void curr_status() {

        //kinematics_data
        Log.i("L_KITT","kinematics_data");
        Kinematics lctn = api.getRobotKinematics();
        Kinematics trs_lctn = api.getTrustedRobotKinematics();

        //printing_position
        Log.i("L_KITT","current_location_cnfds="+lctn.getConfidence()+"_posx="+lctn.getPosition().getX()+"_posy="+lctn.getPosition().getY()+"_posz="+lctn.getPosition().getZ());
        Log.i("L_KITT","trusted_current_location_cnfds="+trs_lctn.getConfidence()+"_posx="+trs_lctn.getPosition().getX()+"_posy="+trs_lctn.getPosition().getY()+"_posz="+trs_lctn.getPosition().getZ());

        //printing_Quaternion
        Log.i("L_KITT","current_quaternion__qua_x="+lctn.getOrientation().getX()+"_qua_y="+lctn.getOrientation().getY()+"_qua_z="+lctn.getOrientation().getZ()+"_qua_w="+lctn.getOrientation().getW());
        Log.i("L_KITT","trusted_current_quaternion__qua_x="+trs_lctn.getOrientation().getX()+"_qua_y="+trs_lctn.getOrientation().getY()+"_qua_z="+trs_lctn.getOrientation().getZ()+"_qua_w="+trs_lctn.getOrientation().getW());

        //printing_linear_velocity_acceleration_angular_velocity
        Log.i("L_KITT","linear_v="+ Arrays.toString(lctn.getLinearVelocity().toArray()) +"_angular_v="+ Arrays.toString(lctn.getAngularVelocity().toArray()) +"_linear_a="+ Arrays.toString(lctn.getLinearAcceleration().toArray()));
        Log.i("L_KITT","trusted_linear_v="+ Arrays.toString(trs_lctn.getLinearVelocity().toArray()) +"_angular_v="+ Arrays.toString(trs_lctn.getAngularVelocity().toArray()) +"_linear_a="+ Arrays.toString(trs_lctn.getLinearAcceleration().toArray()));

        //imu_data
        Log.i("L_KITT","IMU_data");
        ImuResult imu_info= api.getImu();
        Log.i("L_KITT","angular_v="+ Arrays.toString(imu_info.getAngularVelocity().toArray()) +"_linear_a"+ Arrays.toString(imu_info.getLinearAcceleration().toArray()) +"_orientation="+ Arrays.toString(imu_info.getOrientation().toArray()));
        //orientation_is_not_in_details


    }
}