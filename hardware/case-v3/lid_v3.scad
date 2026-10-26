// Spine Smart v3 — revised prototype for stage I.
// Nominal component envelopes from Sizes-1.pdf, not measured purchased parts.
// Units: mm. Print bottom and lid separately; do not print assembled view.
$fn=64;
mode=2; // 0 assembled, 1 bottom, 2 lid on bed, 3 exploded, 4 component plan
L=70; W=58; H=9.6;
wall=1.3; floor_t=1.3; lid_t=1.3; base_h=H-lid_t; corner=5;
clip_x=[10,60]; clip_width=7; clip_t=0.7; clip_gap=0.25;
clip_length=4.5; catch_depth=0.30;
// Adjustable connector openings: verify alignment with actual modules.
charge_x=41.75; esp_usb_x=41; usb_width=12; usb_height=5; usb_z=2.0;
switch_x=18; switch_slot_w=11; switch_slot_h=3.5; switch_z=3.0;

module rounded_box(l,w,h,r) {
    linear_extrude(height=h)
        hull() for(x=[r,l-r],y=[r,w-r]) translate([x,y]) circle(r=r);
}

module board_seat(x,y,w,l,front_open=false,back_open=false) {
    // Adhesive seat, not an unverified screw-hole pattern.
    translate([x,y,floor_t-0.02]) cube([w,l,0.62]);
    translate([x-0.65,y+2,floor_t-0.02]) cube([0.55,l-4,1.22]);
    translate([x+w+0.1,y+2,floor_t-0.02]) cube([0.55,l-4,1.22]);
    if(!front_open) translate([x+2,y-0.65,floor_t-0.02]) cube([w-4,0.55,1.22]);
    if(!back_open) translate([x+2,y+l+0.1,floor_t-0.02]) cube([w-4,0.55,1.22]);
}

module bottom() {
    union() {
        difference() {
            union() {
                rounded_box(L,W,base_h,corner);
                // Closed strap loops: through holes along Z, linked at both ends.
                translate([-5,(W-34)/2,0]) rounded_box(6.1,34,base_h,1.1);
                translate([L-1.1,(W-34)/2,0]) rounded_box(6.1,34,base_h,1.1);
            }
            translate([wall,wall,floor_t])
                rounded_box(L-2*wall,W-2*wall,base_h+0.2,corner-wall);
            translate([-3.6,(W-26)/2,-0.1]) cube([2.8,26,base_h+0.2]);
            translate([L+0.8,(W-26)/2,-0.1]) cube([2.8,26,base_h+0.2]);
            // Four local snap pockets; solid wall above each pocket.
            for(xx=clip_x) {
                translate([xx-3.8,wall-0.45,base_h-4.0]) cube([7.6,0.75,1.4]);
                translate([xx-3.8,W-wall-0.30,base_h-4.0]) cube([7.6,0.75,1.4]);
            }
            // Charging USB-C and programming USB-C, on opposite ends.
            translate([charge_x-usb_width/2,-0.1,usb_z]) cube([usb_width,wall+0.2,usb_height]);
            translate([esp_usb_x-usb_width/2,W-wall-0.1,usb_z]) cube([usb_width,wall+0.2,usb_height]);
            // Switch actuator opening. Tune to the actual purchased switch.
            translate([switch_x,W-wall-0.1,switch_z]) cube([switch_slot_w,wall+0.2,switch_slot_h]);
            // Nail recess, away from ports, under the lid edge.
            translate([17,W-wall-0.1,base_h-0.8]) cube([9,wall+0.2,1.0]);
        }
        // Battery / electronics separator; end gaps allow wires around it.
        translate([29.2,8,floor_t-0.02]) cube([1.2,40,base_h-floor_t-0.6]);
        // Battery locating stops; mounting with a thin adhesive strip.
        translate([2.2,11,floor_t-0.02]) cube([0.6,35,0.82]);
        translate([3,10.2,floor_t-0.02]) cube([25,0.6,0.82]);
        translate([3,46.2,floor_t-0.02]) cube([25,0.6,0.82]);
        board_seat(33,2,17.5,28,true,false); // TP4056, connector faces front
        board_seat(32,32.8,18,22.5,false,true); // ESP32-C3, USB faces rear
        board_seat(51,33,16,21); // MPU6050, direct soldering, no pin headers
        // Coin motor locator. Inner diameter allows a nominal 10 mm motor.
        translate([59,16,floor_t-0.02]) difference() {
            cylinder(d=12,h=1.02);
            translate([0,0,-0.1]) cylinder(d=10.6,h=1.3);
        }
        // Free mounting seats for driver passives and switch.
        translate([53,24,floor_t-0.02]) cube([13,6,0.62]);
        translate([17,49.5,floor_t-0.02]) cube([13,6.5,0.62]);
    }
}

module front_clip(xx) {
    translate([xx-clip_width/2,wall+clip_gap,-clip_length])
        cube([clip_width,clip_t,clip_length+0.04]);
    // Symmetric ramps allow both closing and deliberate reopening.
    translate([xx-clip_width/2,0,0]) rotate([90,0,90])
        linear_extrude(height=clip_width)
            polygon([[wall+clip_gap+0.02,-3.95],
                     [wall-catch_depth,-3.55],
                     [wall-catch_depth,-3.05],
                     [wall+clip_gap+0.02,-2.65]]);
}

module lid() {
    union() {
        rounded_box(L,W,lid_t,corner);
        for(xx=clip_x) {
            front_clip(xx);
            translate([0,W,0]) mirror([0,1,0]) front_clip(xx);
        }
        // Small guides in clear side margins, away from battery and boards.
        for(yy=[6,51]) {
            translate([wall+0.25,yy-1,-0.8]) cube([0.6,2,0.84]);
            translate([L-wall-0.85,yy-1,-0.8]) cube([0.6,2,0.84]);
        }
    }
}

module components() {
    // Heights represent full nominal module envelopes, not just PCB thickness.
    color("orange") translate([3,11,floor_t+0.2]) cube([25,35,6]);
    color("teal") translate([33,2,floor_t+0.8]) cube([17.5,28,4]);
    color("royalblue") translate([32,32.8,floor_t+0.8]) cube([18,22.5,3.8]);
    color("purple") translate([51,33,floor_t+0.8]) cube([16,21,2.8]);
    color("hotpink") translate([59,16,floor_t+0.2]) cylinder(d=10,h=3);
    // Switch outline is an allocation only: it must be measured.
    color("gray") translate([17.5,50,floor_t+0.8]) cube([12,6,5]);
}

if(mode==0) {
    color([0.2,0.35,0.45]) bottom();
    color([0.45,0.6,0.7]) translate([0,0,base_h]) lid();
} else if(mode==1) bottom();
else if(mode==2) translate([0,W,lid_t]) rotate([180,0,0]) lid();
else if(mode==3) {
    color([0.2,0.35,0.45,0.65]) bottom();
    components();
    color([0.45,0.6,0.7,0.8]) translate([0,0,base_h+16]) lid();
} else if(mode==4) {
    color([0.2,0.35,0.45,0.35]) bottom();
    components();
}
