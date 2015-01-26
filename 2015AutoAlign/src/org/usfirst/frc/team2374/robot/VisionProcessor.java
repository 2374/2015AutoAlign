package org.usfirst.frc.team2374.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.ColorMode;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.MeasurementType;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.vision.AxisCamera.Resolution;

public class VisionProcessor {
	int session;
    Image frame;
    AxisCamera camera;
    NIVision.StructuringElement se;
    
    public VisionProcessor() {
    	
    	
        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // open the camera at the IP address assigned. This is the IP address that the camera
        // can be accessed through the web interface.
        camera = new AxisCamera("10.23.74.11");
        
    }

    public VisionReport processCamera() {

        NIVision.Range hue = new NIVision.Range(90,150);
        NIVision.Range s = new NIVision.Range(40,150);
        NIVision.Range l = new NIVision.Range(20, 80);
        
        hue=new NIVision.Range(30,60);
        s=new NIVision.Range(90,255);
        l=new NIVision.Range(0, 250);
        
        NIVision.Image binImage = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 100);
        
        camera.writeResolution(Resolution.k320x240);
        
        camera.getImage(frame);
        NIVision.imaqDrawShapeOnImage(frame, frame, new NIVision.Rect(0, 0, 240, 320),
        		DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 0.0f);

        NIVision.imaqColorThreshold(binImage, frame, 255, ColorMode.HSL, hue, s, l);

        se=new NIVision.StructuringElement(3, 3, 0);

        int rectSize=5;
        for(int i=120; i<240; i+=2*rectSize){ //changed to i++ from i+=2RectSize
        		NIVision.imaqDrawShapeOnImage(binImage, binImage, new NIVision.Rect(i, 0, rectSize, 320),
        				DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 0f);
        }
        int particleCount=NIVision.imaqCountParticles(binImage, 0);
        double area = 0;
        double x, y, w, h;
        x=w=h=0;
        y=130;
        for(int i=0; i<particleCount; ++i){
        	double y2=NIVision.imaqMeasureParticle(binImage, i, 0, MeasurementType.MT_BOUNDING_RECT_TOP);
        	double area2=NIVision.imaqMeasureParticle(binImage, i, 0, MeasurementType.MT_AREA);
        	double w2=NIVision.imaqMeasureParticle(binImage, i, 0, MeasurementType.MT_BOUNDING_RECT_WIDTH);
        	double h2=NIVision.imaqMeasureParticle(binImage, i, 0, MeasurementType.MT_BOUNDING_RECT_HEIGHT);
        	if(h2>3&&w2>50&&y2>y){
        		area=area2;
        		x=NIVision.imaqMeasureParticle(binImage, i, 0, MeasurementType.MT_BOUNDING_RECT_LEFT);
        		y=y2;
        		w=w2;
        		h=h2;
        	}
        }

        //debugging purposes
        SmartDashboard.putNumber("Particle Count", particleCount);
        SmartDashboard.putNumber("Particle Area", area);
        SmartDashboard.putNumber("X",x);
        SmartDashboard.putNumber("Y",y);
        SmartDashboard.putNumber("W", w);
        SmartDashboard.putNumber("H", h);


        float color=255;

        VisionReport v=new VisionReport(x,y,w,h);
        if(Math.abs(v.horizontalOffset)<0.3)color=255*256;

        NIVision.imaqDrawShapeOnImage(frame, frame, new NIVision.Rect(240-(int)y, (int)x, (int)(y-120)*2, (int)w),
        		DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, color);

        CameraServer.getInstance().setImage(binImage);
        if(w==0)return null;
        return v;
        
    }
}