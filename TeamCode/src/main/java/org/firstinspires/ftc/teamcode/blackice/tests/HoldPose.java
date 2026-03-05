package org.firstinspires.ftc.teamcode.blackice.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class HoldPose extends OpMode {
    Follower follower;
    
    Pose startingPose = new Pose(0, 0, 0);
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
        
        follower.setTelemetry(telemetry);
    }
    
    @Override
    public void loop() {
        follower.update();
        
        follower.holdPose(startingPose);
        
        follower.log();
    }
}


//2026-03-03 21:40:07.709 distance from target: 16.905022087998276
//    2026-03-03 21:40:07.728 power: 0.9999027720483014
//    2026-03-03 21:40:07.729 distance from target: 15.63800913893332
//    2026-03-03 21:40:07.755 power: 0.8591094293550526
//    2026-03-03 21:40:07.756 distance from target: 14.015473613588831
//    2026-03-03 21:40:07.779 power: 0.4397768219742942
//    2026-03-03 21:40:07.780 distance from target: 12.55905463751845
//    2026-03-03 21:40:07.800 power: 0.24208389786731552
//    2026-03-03 21:40:07.800 distance from target: 11.247721514363924
//    2026-03-03 21:40:07.821 power: -0.06281503821289493
//    2026-03-03 21:40:07.822 distance from target: 9.967855138102856
//    2026-03-03 21:40:07.841 power: -0.05454130767909072
//    2026-03-03 21:40:07.842 distance from target: 8.800766832246552
//    2026-03-03 21:40:07.861 power: 0.04286734626262211
//    2026-03-03 21:40:07.862 distance from target: 7.708778201125739
//    2026-03-03 21:40:07.888 power: -0.01666678721603031
//    2026-03-03 21:40:07.889 distance from target: 6.617255744032967
//    2026-03-03 21:40:07.910 power: -0.21967449149983959
//    2026-03-03 21:40:07.911 distance from target: 5.420356329970474
//    2026-03-03 21:40:07.929 power: -0.18235704616516296
//    2026-03-03 21:40:07.930 distance from target: 4.581723671259844
//    2026-03-03 21:40:07.948 power: 0.11012829950610406
//    2026-03-03 21:40:07.948 distance from target: 3.9681281142347373
//    2026-03-03 21:40:07.973 power: 0.07400369507479992
//    2026-03-03 21:40:07.974 distance from target: 3.3477033479945817
//    2026-03-03 21:40:07.992 power: -0.21444347715531586
//    2026-03-03 21:40:07.992 distance from target: 2.8164091335506924
//    2026-03-03 21:40:08.010 power: -0.3161116891471564
//    2026-03-03 21:40:08.011 distance from target: 2.226948895792326
//    2026-03-03 21:40:08.028 power: -0.0014774810525536599
//    2026-03-03 21:40:08.029 distance from target: 1.8241024467888778
//    2026-03-03 21:40:08.046 power: 0.11092673357866534
//    2026-03-03 21:40:08.046 distance from target: 1.6156409556471445
//    2026-03-03 21:40:08.066 power: -0.004991666606510984
//    2026-03-03 21:40:08.066 distance from target: 1.3650411771038335
//    2026-03-03 21:40:08.087 power: -0.1530357269462228
//    2026-03-03 21:40:08.088 distance from target: 0.992851677842026
//    2026-03-03 21:40:08.109 power: -0.05917447221912497
//    2026-03-03 21:40:08.110 distance from target: 0.7016515055979298
//    2026-03-03 21:40:08.131 power: 0.017943423418482256
//    2026-03-03 21:40:08.132 distance from target: 0.5533745232529554
//    2026-03-03 21:40:08.150 power: 0.0024917202602406474
//    2026-03-03 21:40:08.151 distance from target: 0.4561075525959595
//    2026-03-03 21:40:08.170 power: -0.023778355773408977
//    2026-03-03 21:40:08.171 distance from target: 0.33128825510580384
//    2026-03-03 21:40:08.188 power: -6.221949075455082E-4
//    2026-03-03 21:40:08.188 distance from target: 0.262856791338578
//    2026-03-03 21:40:08.206 power: 0.02287327881859205
//    2026-03-03 21:40:08.206 distance from target: 0.23527562899852228
//    2026-03-03 21:40:08.229 power: 0.022307381031783372
//    2026-03-03 21:40:08.230 distance from target: 0.20736766424704456
//    2026-03-03 21:40:08.257 power: 0.016975707081916334
//    2026-03-03 21:40:08.259 distance from target: 0.1704582154281411
//    2026-03-03 21:40:08.286 power: 0.03334699280154708


//2026-03-03 21:42:42.241 distance from target: 15.89349413293553
//    2026-03-03 21:42:42.256 power: 0.9997624040803202
//    2026-03-03 21:42:42.257 distance from target: 14.919928122693157
//    2026-03-03 21:42:42.272 power: 0.8696764570880087
//    2026-03-03 21:42:42.273 distance from target: 13.933078555610237
//    2026-03-03 21:42:42.294 power: 0.7995160968146013
//    2026-03-03 21:42:42.295 distance from target: 12.678750826617865
//    2026-03-03 21:42:42.314 power: 0.4664685728923068
//    2026-03-03 21:42:42.315 distance from target: 11.372829166922976
//    2026-03-03 21:42:42.335 power: 0.2220302414095305
//    2026-03-03 21:42:42.335 distance from target: 10.160159133550685
//    2026-03-03 21:42:42.355 power: 0.1152320844572792
//    2026-03-03 21:42:42.356 distance from target: 8.906023641270913
//    2026-03-03 21:42:42.379 power: -0.22629438578139371
//    2026-03-03 21:42:42.380 distance from target: 7.653796098363678
//    2026-03-03 21:42:42.407 power: -0.32888380052954364
//    2026-03-03 21:42:42.408 distance from target: 5.957725224532474
//    2026-03-03 21:42:42.428 power: -0.15170746170701022
//    2026-03-03 21:42:42.428 distance from target: 4.871164877583659
//    2026-03-03 21:42:42.448 power: -0.2881932003478367
//    2026-03-03 21:42:42.448 distance from target: 3.9918933778297188
//    2026-03-03 21:42:42.469 power: -0.22476674072808364
//    2026-03-03 21:42:42.469 distance from target: 3.1815387395423187
//    2026-03-03 21:42:42.489 power: -0.2401637484803837
//    2026-03-03 21:42:42.490 distance from target: 2.394127552903541
//    2026-03-03 21:42:42.511 power: -0.03436138800298941
//    2026-03-03 21:42:42.512 distance from target: 1.7428151528666262
//    2026-03-03 21:42:42.543 power: -0.02566915286007542
//    2026-03-03 21:42:42.543 distance from target: 1.2063497708538335
//    2026-03-03 21:42:42.566 power: -0.15715198039022035
//    2026-03-03 21:42:42.566 distance from target: 0.796794260580711
//    2026-03-03 21:42:42.585 power: -0.10380593390897912
//    2026-03-03 21:42:42.586 distance from target: 0.474187415415841
//    2026-03-03 21:42:42.606 power: -0.02905644836100218
//    2026-03-03 21:42:42.606 distance from target: 0.27714478500245576
//    2026-03-03 21:42:42.628 power: -0.04215963393750442
//    2026-03-03 21:42:42.629 distance from target: 0.14600089966781127
//    2026-03-03 21:42:42.657 power: 0.05584116405972696
//    2026-03-03 21:42:42.659 distance from target: -0.01703505628690749
//    2026-03-03 21:42:42.683 power: 0.019891747692419953
//    2026-03-03 21:42:42.684 distance from target: -0.06223951925443316
//    2026-03-03 21:42:42.706 power: 0.013128004263037247

//[PEDRO/followPath] Started PathChain. size=2 maxPower=1.0 holdEnd=true
//    [PEDRO/DRIVE] t=0.000 normPwr=-0.000 normUsed=-0.000 rawTan=1.000 tanPwr=1.000 tanUsed=1.000 hdgPwr=0.010 hdgUsed=0.010 rem1=1.000 rem2=1.000 drive=[1.000,-0.000] vel=[0.000,0.000] isInter=true reachedEnd=false
//    [PEDRO/SKIP] chainIdx=0 t=0.015 rawTan=1.000 maxPwr=1.000 isAtEnd=false willSkip=false stuckTimer=null
//    [PEDRO/SKIP] chainIdx=0 t=0.067 rawTan=1.000 maxPwr=1.000 isAtEnd=false willSkip=false stuckTimer=null
//    [PEDRO/DRIVE] t=0.164 normPwr=-0.035 normUsed=-0.035 rawTan=1.000 tanPwr=1.000 tanUsed=0.998 hdgPwr=0.045 hdgUsed=0.045 rem1=0.999 rem2=0.998 drive=[0.998,-0.054] vel=[45.863,0.514] isInter=true reachedEnd=false
//    [PEDRO/DRIVE] t=0.275 normPwr=-0.030 normUsed=-0.030 rawTan=1.000 tanPwr=1.000 tanUsed=0.999 hdgPwr=0.027 hdgUsed=0.027 rem1=1.000 rem2=0.999 drive=[0.999,-0.046] vel=[54.679,-0.854] isInter=true reachedEnd=false
//    [PEDRO/DRIVE] t=0.426 normPwr=0.003 normUsed=0.003 rawTan=1.000 tanPwr=1.000 tanUsed=0.999 hdgPwr=-0.034 hdgUsed=-0.034 rem1=1.000 rem2=0.999 drive=[0.999,0.014] vel=[63.476,-0.451] isInter=true reachedEnd=false
//    [PEDRO/DRIVE] t=0.583 normPwr=0.023 normUsed=0.023 rawTan=1.000 tanPwr=1.000 tanUsed=0.999 hdgPwr=-0.034 hdgUsed=-0.034 rem1=1.000 rem2=0.999 drive=[0.999,0.043] vel=[70.011,0.369] isInter=true reachedEnd=false
//    [PEDRO/SKIP] chainIdx=0 t=0.726 rawTan=1.000 maxPwr=1.000 isAtEnd=false willSkip=false stuckTimer=null
//    [PEDRO/DRIVE] t=0.895 normPwr=-0.017 normUsed=-0.017 rawTan=1.000 tanPwr=1.000 tanUsed=1.000 hdgPwr=-0.017 hdgUsed=-0.017 rem1=1.000 rem2=1.000 drive=[1.000,-0.011] vel=[73.202,3.067] isInter=true reachedEnd=false
//    [PEDRO/ADVANCE] Advancing from path 0 → 1 reason: isAtEnd=true nextWithin=false stuck=false
//    [PEDRO/SKIP] chainIdx=1 t=0.003 rawTan=4.849 maxPwr=1.000 isAtEnd=false willSkip=false stuckTimer=null
//    [PEDRO/DRIVE] t=0.016 normPwr=-0.800 normUsed=-0.800 rawTan=4.572 tanPwr=4.572 tanUsed=0.596 hdgPwr=-0.065 hdgUsed=-0.065 rem1=0.600 rem2=0.596 drive=[-0.772,-0.632] vel=[51.656,-5.202] isInter=false reachedEnd=false
//    [PEDRO/SKIP] chainIdx=1 t=0.069 rawTan=4.074 maxPwr=1.000 isAtEnd=false willSkip=false stuckTimer=null
//    [PEDRO/DRIVE] t=0.153 normPwr=-0.800 normUsed=-0.800 rawTan=3.562 tanPwr=3.562 tanUsed=0.548 hdgPwr=-0.245 hdgUsed=-0.245 rem1=0.600 rem2=0.548 drive=[-0.694,-0.676] vel=[28.277,-17.131] isInter=false reachedEnd=false
//    [PEDRO/DRIVE] t=0.252 normPwr=-0.800 normUsed=-0.800 rawTan=2.985 tanPwr=2.985 tanUsed=0.574 hdgPwr=-0.174 hdgUsed=-0.174 rem1=0.600 rem2=0.574 drive=[-0.710,-0.683] vel=[13.696,-23.433] isInter=false reachedEnd=false
//    [PEDRO/DRIVE] t=0.376 normPwr=-0.800 normUsed=-0.800 rawTan=2.302 tanPwr=2.302 tanUsed=0.599 hdgPwr=-0.037 hdgUsed=-0.037 rem1=0.600 rem2=0.599 drive=[-0.770,-0.637] vel=[1.437,-27.947] isInter=false reachedEnd=false
//    [PEDRO/DRIVE] t=0.500 normPwr=-0.800 normUsed=-0.800 rawTan=1.853 tanPwr=1.853 tanUsed=0.599 hdgPwr=0.037 hdgUsed=0.037 rem1=0.600 rem2=0.599 drive=[-0.806,-0.590] vel=[-17.073,-23.010] isInter=false reachedEnd=false
//    [PEDRO/DRIVE] t=0.616 normPwr=-0.800 normUsed=-0.800 rawTan=1.300 tanPwr=1.300 tanUsed=0.598 hdgPwr=0.050 hdgUsed=0.050 rem1=0.600 rem2=0.598 drive=[-0.818,-0.573] vel=[-27.957,-22.263] isInter=false reachedEnd=false
//    [PEDRO/DRIVE] t=0.727 normPwr=-0.800 normUsed=-0.800 rawTan=0.737 tanPwr=0.737 tanUsed=0.598 hdgPwr=0.046 hdgUsed=0.046 rem1=0.600 rem2=0.598 drive=[-0.817,-0.575] vel=[-30.911,-23.290] isInter=false reachedEnd=false
//    [PEDRO/DRIVE] t=0.838 normPwr=-0.666 normUsed=-0.666 rawTan=0.244 tanPwr=0.244 tanUsed=0.244 hdgPwr=0.032 hdgUsed=0.032 rem1=0.746 rem2=0.745 drive=[-0.671,-0.231] vel=[-36.358,-21.964] isInter=false reachedEnd=false
//    [PEDRO/DRIVE] t=0.938 normPwr=0.099 normUsed=0.099 rawTan=0.009 tanPwr=0.009 tanUsed=0.009 hdgPwr=0.030 hdgUsed=0.030 rem1=0.995 rem2=0.995 drive=[0.099,-0.011] vel=[-36.089,-12.901] isInter=false reachedEnd=false
//    [PEDRO/DRIVE] t=0.983 normPwr=0.114 normUsed=0.114 rawTan=0.024 tanPwr=0.024 tanUsed=0.000 hdgPwr=0.017 hdgUsed=0.017 rem1=0.994 rem2=0.993 drive=[0.114,-0.002] vel=[-15.422,-2.860] isInter=false reachedEnd=false
//    [PEDRO/DRIVE] t=0.987 normPwr=0.009 normUsed=0.009 rawTan=0.066 tanPwr=0.066 tanUsed=0.000 hdgPwr=-0.022 hdgUsed=-0.022 rem1=1.000 rem2=1.000 drive=[0.009,0.000] vel=[-1.577,0.294] isInter=false reachedEnd=false
//    [PEDRO/STUCK] Zero-velocity timer started. t=0.987 vel=0.652
