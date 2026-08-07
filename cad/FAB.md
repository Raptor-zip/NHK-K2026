# 製作データ（`out/fab/`）

切り抜き 140 / 3Dプリント 31 / **作れない 2** / 購入・対象外 1028

> 自校の加工能力: アルミ板は**2D 切り抜き + 端面の横穴**まで。**曲げ（板金）と 3D 削り出しはできない。** PETG は 3D プリント。

## 切り抜き（DXF）

レイヤ **OUTLINE**＝外形、**HOLE**＝穴。単位 mm、原点は板の重心。

⚠ 「向き」の列は**どちら側から見た図か**。左右対称の対（`*_L` / `*_R`）を裏返しに切ると鏡像の部品ができる。

| 部品 | 材質 | 板厚 | 外形 mm | 面積 mm² | 穴 | 質量 g | 向き | ファイル |
|---|---|---:|---|---:|---:|---:|---|---|
| hop_floor | PP_DANPLA | t4 | 620×420 | 262,959 | 0 | 210.4 | 法線 (-0.139, -0.000, +0.990) から見た図 | `fab/dxf/hop_floor.dxf` |
| deck_plate | TEKCELL | t5 | 430×430 | 184,900 | 0 | 184.9 | +Z から見た図 | `fab/dxf/deck_plate.dxf` |
| skirt_L | PC | t0.8 | 840×110 | 92,400 | 0 | 88.7 | +Y から見た図 | `fab/dxf/skirt_L.dxf` |
| skirt_R | PC | t0.8 | 840×110 | 92,400 | 0 | 88.7 | +Y から見た図 | `fab/dxf/skirt_R.dxf` |
| skirt_front | PC | t0.8 | 720×110 | 76,600 | 0 | 73.5 | +X から見た図 | `fab/dxf/skirt_front.dxf` |
| skirt_rear | PC | t0.8 | 720×110 | 76,600 | 0 | 73.5 | +X から見た図 | `fab/dxf/skirt_rear.dxf` |
| hop_side_L | PP_DANPLA | t4 | 420×160 | 67,116 | 0 | 53.7 | +Y から見た図 | `fab/dxf/hop_side_L.dxf` |
| hop_side_R | PP_DANPLA | t4 | 420×160 | 67,116 | 0 | 53.7 | +Y から見た図 | `fab/dxf/hop_side_R.dxf` |
| hop_front | PP_DANPLA | t4 | 628×99 | 62,172 | 0 | 49.7 | +X から見た図 | `fab/dxf/hop_front.dxf` |
| press_plate | A5052 | t2 | 380×220 | 45,288 | 27 | 242.7 | +Z から見た図 | `fab/dxf/press_plate.dxf` |
| hop_back | PP_DANPLA | t4 | 628×71 | 44,571 | 0 | 35.7 | +X から見た図 | `fab/dxf/hop_back.dxf` |
| bucket_seat | A5052 | t3 | 330×278 | 36,921 | 9 | 296.8 | +Z から見た図 | `fab/dxf/bucket_seat.dxf` |
| pitch_side_L | A5052 | t3 | 265×221 | 36,042 | 12 | 289.8 | +Y から見た図 | `fab/dxf/pitch_side_L.dxf` |
| yaw_arm_r | A5052 | t6 | 629×60 | 25,236 | 12 | 405.8 | +Z から見た図 | `fab/dxf/yaw_arm_r.dxf` |
| lift_guide | A5052 | t1.5 | 562×62 | 25,112 | 0 | 101.0 | 法線 (-0.838, +0.000, +0.546) から見た図 | `fab/dxf/lift_guide.dxf` |
| press_shelf | A5052 | t3 | 494×56 | 23,797 | 6 | 191.3 | +Z から見た図 | `fab/dxf/press_shelf.dxf` |
| yaw_arm_f | A5052 | t6 | 629×60 | 21,784 | 16 | 350.3 | +Z から見た図 | `fab/dxf/yaw_arm_f.dxf` |
| ramp_guide | A5052 | t1 | 579×30 | 21,781 | 0 | 58.4 | 法線 (-0.810, -0.000, +0.586) から見た図 | `fab/dxf/ramp_guide.dxf` |
| pitch_side_R | A5052 | t3 | 160×158 | 19,268 | 14 | 154.9 | +Y から見た図 | `fab/dxf/pitch_side_R.dxf` |
| rail_plate_L | A5052 | t4 | 465×59 | 19,128 | 6 | 205.0 | +Y から見た図 | `fab/dxf/rail_plate_L.dxf` |
| rail_plate_R | A5052 | t4 | 465×59 | 19,128 | 6 | 205.0 | +Y から見た図 | `fab/dxf/rail_plate_R.dxf` |
| car_beam | A5052 | t3 | 624×40 | 17,995 | 7 | 144.7 | +Z から見た図 | `fab/dxf/car_beam.dxf` |
| fork_root | A5052 | t3 | 560×28 | 15,680 | 0 | 126.1 | +Z から見た図 | `fab/dxf/fork_root.dxf` |
| car_side_L | A5052 | t5 | 350×60 | 14,876 | 5 | 199.3 | +Y から見た図 | `fab/dxf/car_side_L.dxf` |
| car_side_R | A5052 | t5 | 350×60 | 14,876 | 5 | 199.3 | +Y から見た図 | `fab/dxf/car_side_R.dxf` |
| mount_brk_fl | A5052 | t4 | 130×120 | 12,443 | 13 | 133.4 | +Y から見た図 | `fab/dxf/mount_brk_fl.dxf` |
| mount_brk_fr | A5052 | t4 | 130×120 | 12,443 | 13 | 133.4 | +Y から見た図 | `fab/dxf/mount_brk_fr.dxf` |
| mount_brk_rl | A5052 | t4 | 130×120 | 12,443 | 13 | 133.4 | +Y から見た図 | `fab/dxf/mount_brk_rl.dxf` |
| mount_brk_rr | A5052 | t4 | 130×120 | 12,443 | 13 | 133.4 | +Y から見た図 | `fab/dxf/mount_brk_rr.dxf` |
| yaw_side_L | A5052 | t3 | 230×157 | 11,546 | 9 | 92.8 | +Y から見た図 | `fab/dxf/yaw_side_L.dxf` |
| yaw_side_R | A5052 | t3 | 230×159 | 11,504 | 7 | 92.5 | +Y から見た図 | `fab/dxf/yaw_side_R.dxf` |
| fork_root_v | A5052 | t3 | 560×20 | 11,200 | 0 | 90.0 | +X から見た図 | `fab/dxf/fork_root_v.dxf` |
| pedestal_ring1 | A5052 | t4 | 290×70 | 11,140 | 1 | 119.4 | +Z から見た図 | `fab/dxf/pedestal_ring1.dxf` |
| lidar_high_brk | A5052 | t8 | 153×70 | 10,696 | 0 | 229.3 | +X から見た図 | `fab/dxf/lidar_high_brk.dxf` |
| tine0 | SUS304 | t2.5 | 379×20 | 7,313 | 0 | 145.0 | +Z から見た図 | `fab/dxf/tine0.dxf` |
| tine1 | SUS304 | t2.5 | 379×20 | 7,313 | 0 | 145.0 | +Z から見た図 | `fab/dxf/tine1.dxf` |
| tine2 | SUS304 | t2.5 | 379×20 | 7,313 | 0 | 145.0 | +Z から見た図 | `fab/dxf/tine2.dxf` |
| tine3 | SUS304 | t2.5 | 379×20 | 7,313 | 0 | 145.0 | +Z から見た図 | `fab/dxf/tine3.dxf` |
| tine4 | SUS304 | t2.5 | 379×20 | 7,313 | 0 | 145.0 | +Z から見た図 | `fab/dxf/tine4.dxf` |
| pedestal_ring0 | A5052 | t4 | 242×45 | 6,837 | 1 | 73.3 | +Z から見た図 | `fab/dxf/pedestal_ring0.dxf` |
| gus_brace_L_lo | A5052 | t3 | 115×105 | 6,412 | 4 | 51.6 | +Y から見た図 | `fab/dxf/gus_brace_L_lo.dxf` |
| gus_brace_R_lo | A5052 | t3 | 115×105 | 6,412 | 4 | 51.6 | +Y から見た図 | `fab/dxf/gus_brace_R_lo.dxf` |
| car_brk_L | A5052 | t5 | 95×70 | 5,581 | 2 | 74.8 | +Z から見た図 | `fab/dxf/car_brk_L.dxf` |
| car_brk_R | A5052 | t5 | 95×70 | 5,581 | 2 | 74.8 | +Z から見た図 | `fab/dxf/car_brk_R.dxf` |
| gus_brace_L_hi | A5052 | t3 | 107×106 | 5,244 | 4 | 42.2 | +Y から見た図 | `fab/dxf/gus_brace_L_hi.dxf` |
| gus_brace_R_hi | A5052 | t3 | 107×106 | 5,244 | 4 | 42.2 | +Y から見た図 | `fab/dxf/gus_brace_R_hi.dxf` |
| lidar_lvl_base_rear | A5052 | t6 | 84×77 | 5,211 | 1 | 83.8 | +Z から見た図 | `fab/dxf/lidar_lvl_base_rear.dxf` |
| lidar_lvl_base_front | A5052 | t6 | 84×77 | 4,989 | 1 | 80.2 | +Z から見た図 | `fab/dxf/lidar_lvl_base_front.dxf` |
| estop_plate_f | A5052 | t4 | 80×60 | 4,800 | 0 | 51.5 | +Y から見た図 | `fab/dxf/estop_plate_f.dxf` |
| estop_plate_r | A5052 | t4 | 80×60 | 4,800 | 0 | 51.5 | +Z から見た図 | `fab/dxf/estop_plate_r.dxf` |
| press_post_L | A5052 | t6 | 125×50 | 4,657 | 3 | 74.9 | +Y から見た図 | `fab/dxf/press_post_L.dxf` |
| press_post_R | A5052 | t6 | 125×50 | 4,657 | 3 | 74.9 | +Y から見た図 | `fab/dxf/press_post_R.dxf` |
| grab_motor_brk | A5052 | t3 | 70×70 | 4,646 | 1 | 37.4 | +Y から見た図 | `fab/dxf/grab_motor_brk.dxf` |
| lidar_lvl_base_high | A5052 | t6 | 84×69 | 4,539 | 1 | 73.0 | +Z から見た図 | `fab/dxf/lidar_lvl_base_high.dxf` |
| beltbrk_drv_L | A5052 | t3 | 70×60 | 4,087 | 1 | 32.9 | +Y から見た図 | `fab/dxf/beltbrk_drv_L.dxf` |
| beltbrk_drv_R | A5052 | t3 | 70×60 | 4,087 | 1 | 32.9 | +Y から見た図 | `fab/dxf/beltbrk_drv_R.dxf` |
| mount_ear_fl | A5052 | t8 | 120×34 | 4,080 | 0 | 87.5 | +Z から見た図 | `fab/dxf/mount_ear_fl.dxf` |
| mount_ear_fr | A5052 | t8 | 120×34 | 4,080 | 0 | 87.5 | +Z から見た図 | `fab/dxf/mount_ear_fr.dxf` |
| mount_ear_rl | A5052 | t8 | 120×34 | 4,080 | 0 | 87.5 | +Z から見た図 | `fab/dxf/mount_ear_rl.dxf` |
| mount_ear_rr | A5052 | t8 | 120×34 | 4,080 | 0 | 87.5 | +Z から見た図 | `fab/dxf/mount_ear_rr.dxf` |
| roller_mot_stand_d | A5052 | t4 | 70×62 | 4,065 | 1 | 43.6 | +Y から見た図 | `fab/dxf/roller_mot_stand_d.dxf` |
| roller_mot_stand_u | A5052 | t4 | 70×62 | 4,065 | 1 | 43.6 | +Y から見た図 | `fab/dxf/roller_mot_stand_u.dxf` |
| press_face | A5052 | t3 | 70×60 | 3,985 | 1 | 32.0 | +Y から見た図 | `fab/dxf/press_face.dxf` |
| lidar_lvl_top_high | A5052 | t6 | 84×60 | 3,783 | 1 | 60.8 | +Z から見た図 | `fab/dxf/lidar_lvl_top_high.dxf` |
| lidar_lvl_top_rear | A5052 | t6 | 84×60 | 3,783 | 1 | 60.8 | +Z から見た図 | `fab/dxf/lidar_lvl_top_rear.dxf` |
| lidar_lvl_top_front | A5052 | t6 | 84×60 | 3,663 | 1 | 58.9 | +Z から見た図 | `fab/dxf/lidar_lvl_top_front.dxf` |
| yaw_motor_deck | A5052 | t4 | 75×60 | 3,503 | 6 | 37.5 | +Z から見た図 | `fab/dxf/yaw_motor_deck.dxf` |
| brk_yaw_Lfv | A5052 | t8 | 75×45 | 3,375 | 0 | 72.4 | +Y から見た図 | `fab/dxf/brk_yaw_Lfv.dxf` |
| brk_yaw_Lrv | A5052 | t8 | 75×45 | 3,375 | 0 | 72.4 | +Y から見た図 | `fab/dxf/brk_yaw_Lrv.dxf` |
| brk_yaw_Rfv | A5052 | t8 | 75×45 | 3,375 | 0 | 72.4 | +Y から見た図 | `fab/dxf/brk_yaw_Rfv.dxf` |
| brk_yaw_Rrv | A5052 | t8 | 75×45 | 3,375 | 0 | 72.4 | +Y から見た図 | `fab/dxf/brk_yaw_Rrv.dxf` |
| ramp_side_L | A5052 | t1 | 120×96 | 3,240 | 0 | 8.7 | +Y から見た図 | `fab/dxf/ramp_side_L.dxf` |
| ramp_side_R | A5052 | t1 | 120×96 | 3,240 | 0 | 8.7 | +Y から見た図 | `fab/dxf/ramp_side_R.dxf` |
| cam_brk | A5052 | t3 | 60×50 | 3,000 | 0 | 24.1 | +X から見た図 | `fab/dxf/cam_brk.dxf` |
| hop_hanger_L0v | A5052 | t8 | 60×40 | 2,400 | 0 | 51.5 | +Y から見た図 | `fab/dxf/hop_hanger_L0v.dxf` |
| hop_hanger_L1v | A5052 | t8 | 60×40 | 2,400 | 0 | 51.5 | +Y から見た図 | `fab/dxf/hop_hanger_L1v.dxf` |
| hop_hanger_R0v | A5052 | t8 | 60×40 | 2,400 | 0 | 51.5 | +Y から見た図 | `fab/dxf/hop_hanger_R0v.dxf` |
| hop_hanger_R1v | A5052 | t8 | 60×40 | 2,400 | 0 | 51.5 | +Y から見た図 | `fab/dxf/hop_hanger_R1v.dxf` |
| thk_arm_L | A5052 | t3 | 60×40 | 2,400 | 0 | 19.3 | +X から見た図 | `fab/dxf/thk_arm_L.dxf` |
| thk_arm_R | A5052 | t3 | 60×40 | 2,400 | 0 | 19.3 | +X から見た図 | `fab/dxf/thk_arm_R.dxf` |
| beltbrk_idl_L | A5052 | t3 | 87×36 | 2,300 | 3 | 18.5 | +Y から見た図 | `fab/dxf/beltbrk_idl_L.dxf` |
| beltbrk_idl_R | A5052 | t3 | 87×36 | 2,245 | 2 | 18.0 | +Y から見た図 | `fab/dxf/beltbrk_idl_R.dxf` |
| brk_lift_L0 | A5052 | t4 | 46×38 | 2,186 | 0 | 23.4 | 法線 (-0.838, +0.000, +0.546) から見た図 | `fab/dxf/brk_lift_L0.dxf` |
| brk_lift_R0 | A5052 | t4 | 46×38 | 2,186 | 0 | 23.4 | 法線 (-0.838, +0.000, +0.546) から見た図 | `fab/dxf/brk_lift_R0.dxf` |
| belt_clamp_L | A5052 | t30 | 72×58 | 2,022 | 0 | 162.6 | +X から見た図 | `fab/dxf/belt_clamp_L.dxf` |
| belt_clamp_R | A5052 | t30 | 72×58 | 2,022 | 0 | 162.6 | +X から見た図 | `fab/dxf/belt_clamp_R.dxf` |
| brk_mast_cross_Lm | A5052 | t6 | 64×50 | 1,880 | 0 | 30.2 | +X から見た図 | `fab/dxf/brk_mast_cross_Lm.dxf` |
| brk_mast_cross_Lp | A5052 | t6 | 64×50 | 1,880 | 0 | 30.2 | +X から見た図 | `fab/dxf/brk_mast_cross_Lp.dxf` |
| brk_mast_cross_Rm | A5052 | t6 | 64×50 | 1,880 | 0 | 30.2 | +X から見た図 | `fab/dxf/brk_mast_cross_Rm.dxf` |
| brk_mast_cross_Rp | A5052 | t6 | 64×50 | 1,880 | 0 | 30.2 | +X から見た図 | `fab/dxf/brk_mast_cross_Rp.dxf` |
| grab_motor_arm | A5052 | t6 | 61×30 | 1,830 | 0 | 29.4 | +Z から見た図 | `fab/dxf/grab_motor_arm.dxf` |
| chair_mount_0 | A5052 | t15 | 50×40 | 1,400 | 1 | 56.3 | +Z から見た図 | `fab/dxf/chair_mount_0.dxf` |
| chair_mount_1 | A5052 | t15 | 50×40 | 1,400 | 1 | 56.3 | +Z から見た図 | `fab/dxf/chair_mount_1.dxf` |
| chair_mount_2 | A5052 | t15 | 50×40 | 1,400 | 1 | 56.3 | +Z から見た図 | `fab/dxf/chair_mount_2.dxf` |
| chair_mount_3 | A5052 | t15 | 50×40 | 1,400 | 1 | 56.3 | +Z から見た図 | `fab/dxf/chair_mount_3.dxf` |
| nip_wedge_L | A5052 | t9 | 70×20 | 1,400 | 0 | 33.8 | 法線 (+0.050, -0.000, +0.999) から見た図 | `fab/dxf/nip_wedge_L.dxf` |
| nip_wedge_R | A5052 | t9 | 70×20 | 1,400 | 0 | 33.8 | 法線 (+0.050, -0.000, +0.999) から見た図 | `fab/dxf/nip_wedge_R.dxf` |
| ramp_seat_Lv | A5052 | t6 | 70×20 | 1,400 | 0 | 22.5 | +Y から見た図 | `fab/dxf/ramp_seat_Lv.dxf` |
| ramp_seat_Rv | A5052 | t6 | 70×20 | 1,400 | 0 | 22.5 | +Y から見た図 | `fab/dxf/ramp_seat_Rv.dxf` |
| ramp_seat_L | A5052 | t3 | 40×30 | 1,200 | 0 | 9.6 | +Z から見た図 | `fab/dxf/ramp_seat_L.dxf` |
| ramp_seat_R | A5052 | t3 | 40×30 | 1,200 | 0 | 9.6 | +Z から見た図 | `fab/dxf/ramp_seat_R.dxf` |
| ramp_foot_L | A5052 | t6 | 42×26 | 1,092 | 0 | 17.6 | +Y から見た図 | `fab/dxf/ramp_foot_L.dxf` |
| ramp_foot_R | A5052 | t6 | 42×26 | 1,092 | 0 | 17.6 | +Y から見た図 | `fab/dxf/ramp_foot_R.dxf` |
| hop_hanger_L0 | A5052 | t8 | 60×18 | 1,080 | 0 | 23.2 | +Z から見た図 | `fab/dxf/hop_hanger_L0.dxf` |
| hop_hanger_L1 | A5052 | t8 | 60×18 | 1,080 | 0 | 23.2 | +Z から見た図 | `fab/dxf/hop_hanger_L1.dxf` |
| hop_hanger_R0 | A5052 | t8 | 60×18 | 1,080 | 0 | 23.2 | +Z から見た図 | `fab/dxf/hop_hanger_R0.dxf` |
| hop_hanger_R1 | A5052 | t8 | 60×18 | 1,080 | 0 | 23.2 | +Z から見た図 | `fab/dxf/hop_hanger_R1.dxf` |
| car_rib_L | A5052 | t8 | 40×24 | 960 | 0 | 20.6 | +Y から見た図 | `fab/dxf/car_rib_L.dxf` |
| car_rib_R | A5052 | t8 | 40×24 | 960 | 0 | 20.6 | +Y から見た図 | `fab/dxf/car_rib_R.dxf` |
| car_brk_Lv | A5052 | t8 | 70×13 | 910 | 0 | 19.5 | +Y から見た図 | `fab/dxf/car_brk_Lv.dxf` |
| car_brk_Rv | A5052 | t8 | 70×13 | 910 | 0 | 19.5 | +Y から見た図 | `fab/dxf/car_brk_Rv.dxf` |
| car_beam_eL | A5052 | t8 | 40×20 | 800 | 0 | 17.2 | +Y から見た図 | `fab/dxf/car_beam_eL.dxf` |
| car_beam_eR | A5052 | t8 | 40×20 | 800 | 0 | 17.2 | +Y から見た図 | `fab/dxf/car_beam_eR.dxf` |
| yaw_motor_post_m | A5052 | t16 | 40×19 | 760 | 0 | 32.6 | +X から見た図 | `fab/dxf/yaw_motor_post_m.dxf` |
| yaw_motor_post_p | A5052 | t16 | 40×19 | 760 | 0 | 32.6 | +X から見た図 | `fab/dxf/yaw_motor_post_p.dxf` |
| brk_yaw_Lf | A5052 | t5 | 75×8 | 600 | 0 | 8.0 | +Z から見た図 | `fab/dxf/brk_yaw_Lf.dxf` |
| brk_yaw_Lr | A5052 | t5 | 75×8 | 600 | 0 | 8.0 | +Z から見た図 | `fab/dxf/brk_yaw_Lr.dxf` |
| brk_yaw_Rf | A5052 | t5 | 75×8 | 600 | 0 | 8.0 | +Z から見た図 | `fab/dxf/brk_yaw_Rf.dxf` |
| brk_yaw_Rr | A5052 | t5 | 75×8 | 600 | 0 | 8.0 | +Z から見た図 | `fab/dxf/brk_yaw_Rr.dxf` |
| ramp_strut_L | A5052 | t3 | 31×31 | 476 | 0 | 3.8 | +Y から見た図 | `fab/dxf/ramp_strut_L.dxf` |
| ramp_strut_R | A5052 | t3 | 31×31 | 476 | 0 | 3.8 | +Y から見た図 | `fab/dxf/ramp_strut_R.dxf` |
| roller_mot_post_d0 | A5052 | t10 | 46×10 | 465 | 0 | 12.5 | +X から見た図 | `fab/dxf/roller_mot_post_d0.dxf` |
| roller_mot_post_d1 | A5052 | t10 | 46×10 | 465 | 0 | 12.5 | +X から見た図 | `fab/dxf/roller_mot_post_d1.dxf` |
| roller_mot_post_d2 | A5052 | t10 | 46×10 | 465 | 0 | 12.5 | +X から見た図 | `fab/dxf/roller_mot_post_d2.dxf` |
| roller_mot_post_d3 | A5052 | t10 | 46×10 | 465 | 0 | 12.5 | +X から見た図 | `fab/dxf/roller_mot_post_d3.dxf` |
| roller_mot_post_u0 | A5052 | t10 | 46×10 | 465 | 0 | 12.5 | +X から見た図 | `fab/dxf/roller_mot_post_u0.dxf` |
| roller_mot_post_u1 | A5052 | t10 | 46×10 | 465 | 0 | 12.5 | +X から見た図 | `fab/dxf/roller_mot_post_u1.dxf` |
| roller_mot_post_u2 | A5052 | t10 | 46×10 | 465 | 0 | 12.5 | +X から見た図 | `fab/dxf/roller_mot_post_u2.dxf` |
| roller_mot_post_u3 | A5052 | t10 | 46×10 | 465 | 0 | 12.5 | +X から見た図 | `fab/dxf/roller_mot_post_u3.dxf` |
| press_post_Lf0 | A5052 | t3 | 50×8 | 400 | 0 | 3.2 | +Z から見た図 | `fab/dxf/press_post_Lf0.dxf` |
| press_post_Lf1 | A5052 | t3 | 50×8 | 400 | 0 | 3.2 | +Z から見た図 | `fab/dxf/press_post_Lf1.dxf` |
| press_post_Rf0 | A5052 | t3 | 50×8 | 400 | 0 | 3.2 | +Z から見た図 | `fab/dxf/press_post_Rf0.dxf` |
| press_post_Rf1 | A5052 | t3 | 50×8 | 400 | 0 | 3.2 | +Z から見た図 | `fab/dxf/press_post_Rf1.dxf` |
| ramp_foot_Le | A5052 | t3 | 33×8 | 264 | 0 | 2.1 | +X から見た図 | `fab/dxf/ramp_foot_Le.dxf` |
| ramp_foot_Re | A5052 | t3 | 33×8 | 264 | 0 | 2.1 | +X から見た図 | `fab/dxf/ramp_foot_Re.dxf` |
| sing_clip0 | A5052 | t36 | 12×7 | 24 | 0 | 2.3 | +Y から見た図 | `fab/dxf/sing_clip0.dxf` |
| sing_clip1 | A5052 | t36 | 12×7 | 24 | 0 | 2.3 | +Y から見た図 | `fab/dxf/sing_clip1.dxf` |
| sing_clip2 | A5052 | t36 | 12×7 | 24 | 0 | 2.3 | +Y から見た図 | `fab/dxf/sing_clip2.dxf` |
| sing_clip3 | A5052 | t36 | 12×7 | 24 | 0 | 2.3 | +Y から見た図 | `fab/dxf/sing_clip3.dxf` |
| sing_clip4 | A5052 | t36 | 12×7 | 24 | 0 | 2.3 | +Y から見た図 | `fab/dxf/sing_clip4.dxf` |

## 3Dプリント（STL）

PETG。⚠ 造形範囲は 256×256×256mm（Bambu Lab P1S / X1C）、壁はノズル 2 本ぶん 0.8mm を下限として見ている。

| 部品 | 外形 mm | 質量 g | ファイル |
|---|---|---:|---|
| bucket_hold | 222×222×4 | 34.1 | `fab/stl/bucket_hold.stl` |
| bucket_tab_0 | 23×47×16 | 4.1 | `fab/stl/bucket_tab_0.stl` |
| bucket_tab_1 | 47×23×16 | 4.1 | `fab/stl/bucket_tab_1.stl` |
| bucket_tab_2 | 23×47×16 | 4.1 | `fab/stl/bucket_tab_2.stl` |
| bucket_tab_3 | 47×23×16 | 4.1 | `fab/stl/bucket_tab_3.stl` |
| cam_follower | 18×40×18 | 6.1 | `fab/stl/cam_follower.stl` |
| disp_frame | 24×205×132 | 79.3 | `fab/stl/disp_frame.stl` |
| nip_guide_d0 | 40×200×26 | 11.3 | `fab/stl/nip_guide_d0.stl` |
| nip_guide_d1 | 40×200×26 | 12.4 | `fab/stl/nip_guide_d1.stl` |
| nip_guide_d2 | 40×200×26 | 11.9 | `fab/stl/nip_guide_d2.stl` |
| nip_guide_u0 | 40×200×26 | 11.3 | `fab/stl/nip_guide_u0.stl` |
| nip_guide_u1 | 40×200×26 | 12.4 | `fab/stl/nip_guide_u1.stl` |
| nip_guide_u2 | 40×200×26 | 11.9 | `fab/stl/nip_guide_u2.stl` |
| odo_arm0 | 30×34×84 | 5.8 | `fab/stl/odo_arm0.stl` |
| odo_arm1 | 34×30×84 | 5.8 | `fab/stl/odo_arm1.stl` |
| odo_arm2 | 34×30×84 | 5.8 | `fab/stl/odo_arm2.stl` |
| odo_brk0 | 37×49×77 | 11.9 | `fab/stl/odo_brk0.stl` |
| odo_brk1 | 49×37×77 | 11.4 | `fab/stl/odo_brk1.stl` |
| odo_brk2 | 49×37×77 | 11.4 | `fab/stl/odo_brk2.stl` |
| odo_encmnt0 | 9×20×22 | 1.3 | `fab/stl/odo_encmnt0.stl` |
| odo_encmnt1 | 20×9×22 | 1.3 | `fab/stl/odo_encmnt1.stl` |
| odo_encmnt2 | 20×9×22 | 1.3 | `fab/stl/odo_encmnt2.stl` |
| odo_hub0 | 23×30×30 | 2.3 | `fab/stl/odo_hub0.stl` |
| odo_hub1 | 30×23×30 | 2.3 | `fab/stl/odo_hub1.stl` |
| odo_hub2 | 30×23×30 | 2.3 | `fab/stl/odo_hub2.stl` |
| sing_hub0 | 34×30×34 | 12.0 | `fab/stl/sing_hub0.stl` |
| sing_hub1 | 34×30×34 | 12.0 | `fab/stl/sing_hub1.stl` |
| sing_hub2 | 34×30×34 | 12.0 | `fab/stl/sing_hub2.stl` |
| sing_hub3 | 34×30×34 | 12.0 | `fab/stl/sing_hub3.stl` |
| sing_hub4 | 34×30×34 | 12.0 | `fab/stl/sing_hub4.stl` |
| sing_mot_brk | 44×50×44 | 5.8 | `fab/stl/sing_mot_brk.stl` |

## 作れない — 直すもの（STEP は参考）

| 部品 | 材質 | 外形 mm | 質量 g | なぜ切り抜きにできないか |
|---|---|---|---:|---|
| disp_screen | SCREEN | 1×172×108 | 46.4 | SCREEN は板材として買えない（板厚 1.0mm） |
| lift_mot_brk | A5052 | 55×28×50 | 23.0 | 平板でない。**3D の削り出しはできない** |

## 板取り（材質 × 板厚）

| 材質 | 板厚 | 枚数 | 合計面積 mm² | 板 1 枚 | 必要枚数(60%歩留) |
|---|---:|---:|---:|---|---:|
| A5052 | t1 | 3 | 28,262 | 500×1000 | 1 |
| A5052 | t1.5 | 1 | 25,112 | 500×1000 | 1 |
| A5052 | t2 | 1 | 45,288 | 500×1000 | 1 |
| A5052 | t3 | 32 | 241,895 | 500×1000 | 1 |
| A5052 | t4 | 15 | 131,611 | 500×1000 | 1 |
| A5052 | t5 | 8 | 43,315 | 500×1000 | 1 |
| A5052 | t6 | 19 | 96,638 | 500×1000 | 1 |
| A5052 | t8 | 23 | 59,776 | 500×1000 | 1 |
| A5052 | t9 | 2 | 2,800 | 500×1000 | 1 |
| A5052 | t10 | 8 | 3,720 | 500×1000 | 1 |
| A5052 | t15 | 4 | 5,600 | 500×1000 | 1 |
| A5052 | t16 | 2 | 1,520 | 500×1000 | 1 |
| A5052 | t30 | 2 | 4,044 | 500×1000 | 1 |
| A5052 | t36 | 5 | 120 | 500×1000 | 1 |
| PC | t0.8 | 4 | 338,000 | 600×900 | 2 |
| PP_DANPLA | t4 | 5 | 503,934 | 910×1820 | 1 |
| SUS304 | t2.5 | 5 | 36,564 | 500×1000 | 1 |
| TEKCELL | t5 | 1 | 184,900 | 910×1820 | 1 |

## 購入・対象外

| 部品 | 材質 | 理由 |
|---|---|---|
| aim_camera | SENSOR | 照準カメラ（購入） |
| battery | BATTERY | 6S LiPo（購入） |
| belt_grab_L | RUBBER | タイミングベルト（購入） |
| belt_grab_R | RUBBER | タイミングベルト（購入） |
| beltbush_drv_L | STEEL | POM フランジブッシュ（購入） |
| beltbush_drv_R | STEEL | POM フランジブッシュ（購入） |
| beltbush_idl_L | STEEL | POM フランジブッシュ（購入） |
| beltbush_idl_R | STEEL | POM フランジブッシュ（購入） |
| beltpul_drv_L | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| beltpul_drv_R | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| beltpul_idl_L | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| beltpul_idl_R | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| beltshaft_drvL | STEEL | φ10 鋼軸（切断のみ） |
| beltshaft_drvR | STEEL | φ10 鋼軸（切断のみ） |
| beltshaft_idlL | STEEL | φ10 鋼軸（切断のみ） |
| beltshaft_idlR | STEEL | φ10 鋼軸（切断のみ） |
| brace_L | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| brace_R | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| breaker | MOTOR | DJI M3508 / M2006（購入） |
| brk_cross_xm110_mid_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xm110_mid_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xm210_mid_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xm210_mid_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xm410_L_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xm410_L_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xm410_R_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xm410_R_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xm410_mid_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xm410_mid_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp110_mid_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp110_mid_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp210_mid_mn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp210_mid_pn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp410_L_mn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp410_L_pn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp410_R_mn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp410_R_pn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp410_mid_mn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_cross_xp410_mid_pn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_disp_Ld | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_disp_Lu | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_disp_Rd | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_disp_Ru | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_mast_arm_L_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_mast_arm_L_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_mast_arm_R_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_mast_arm_R_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_mast_corner_L | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_mast_corner_R | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_pedestal_beam0_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_pedestal_beam0_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_pedestal_beam1_mn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_pedestal_beam1_pn | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_post_front_L_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_post_front_L_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_post_front_R_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_post_front_R_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_post_rear_L_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_post_rear_L_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_post_rear_R_m | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_post_rear_R_p | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam0_L_pd | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam0_L_pu | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam0_R_pd | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam0_R_pu | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam1_L_md | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam1_L_mu | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam1_L_pd | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam1_R_md | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam1_R_mu | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| brk_topbeam1_R_pd | ADC12 | MISUMI 5 シリーズ ブラケット（購入） |
| bucket | PC | 移動バケツ（審判が置く購入品） |
| bucket_2l_datum | A5052 | 規定 3.2.3c の目盛り位置（実体ではない） |
| cab_drive_fl | CABLE | 配線束 |
| cab_drive_fr | CABLE | 配線束 |
| cab_drive_rl | CABLE | 配線束 |
| cab_drive_rr | CABLE | 配線束 |
| cab_grabber | A5052 | 配線束 |
| cab_lidar_front | CABLE | 配線束 |
| cab_lidar_high | CABLE | 配線束 |
| cab_lidar_rear | CABLE | 配線束 |
| cab_pc_in | CABLE | 配線束 |
| cab_pc_pwr | CABLE | 配線束 |
| cab_power | CABLE | 配線束 |
| cab_saddle_fl0 | A5052 | 配線束 |
| cab_saddle_fl1 | A5052 | 配線束 |
| cab_saddle_fr0 | A5052 | 配線束 |
| cab_saddle_fr1 | A5052 | 配線束 |
| cab_saddle_rl0 | A5052 | 配線束 |
| cab_saddle_rl1 | A5052 | 配線束 |
| cab_saddle_rr0 | A5052 | 配線束 |
| cab_saddle_rr1 | A5052 | 配線束 |
| cab_turret | CABLE | 配線束 |
| cab_usb_disp | CABLE | 配線束 |
| cab_usb_mcu | CABLE | 配線束 |
| chair_5go | PLYWOOD | 新JIS 教室用椅子 5号（購入） |
| cross_xm110_mid | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| cross_xm210_mid | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| cross_xm410_L | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| cross_xm410_R | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| cross_xm410_mid | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| cross_xp110_mid | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| cross_xp210_mid | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| cross_xp410_L | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| cross_xp410_R | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| cross_xp410_mid | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| disp_beam | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| disp_panel | PC | 表示器モジュール（購入。t14 はモジュールの厚み） |
| esc610_0 | MOTOR | DJI M3508 / M2006（購入） |
| esc610_1 | MOTOR | DJI M3508 / M2006（購入） |
| esc610_2 | MOTOR | DJI M3508 / M2006（購入） |
| esc610_3 | MOTOR | DJI M3508 / M2006（購入） |
| esc620_0 | MOTOR | DJI M3508 / M2006（購入） |
| esc620_1 | MOTOR | DJI M3508 / M2006（購入） |
| esc620_2 | MOTOR | DJI M3508 / M2006（購入） |
| esc620_3 | MOTOR | DJI M3508 / M2006（購入） |
| esc620_4 | MOTOR | DJI M3508 / M2006（購入） |
| esc620_5 | MOTOR | DJI M3508 / M2006（購入） |
| esc620_6 | MOTOR | DJI M3508 / M2006（購入） |
| estop_f | ESTOP | 非常停止スイッチ（購入） |
| estop_r | ESTOP | 非常停止スイッチ（購入） |
| fork_clamp0 | STEEL | φ8 シャフトクランプ（購入） |
| fork_clamp1 | STEEL | φ8 シャフトクランプ（購入） |
| fork_clamp2 | STEEL | φ8 シャフトクランプ（購入） |
| fork_clamp3 | STEEL | φ8 シャフトクランプ（購入） |
| fork_clamp4 | STEEL | φ8 シャフトクランプ（購入） |
| grab_belt | RUBBER | タイミングベルト（購入） |
| grab_motor | MOTOR | DJI M3508 / M2006（購入） |
| grab_pul_m | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| grab_pul_s | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| hinge_blk_L | A5052 | φ8 シャフトクランプ（購入） |
| hinge_blk_R | A5052 | φ8 シャフトクランプ（購入） |
| hinge_shaft | STEEL | φ8 鋼軸（切断のみ） |
| hub_fl | A5052 | メカナムのハブアダプタ（ホイールに付属 or 購入） |
| hub_fr | A5052 | メカナムのハブアダプタ（ホイールに付属 or 購入） |
| hub_rl | A5052 | メカナムのハブアダプタ（ホイールに付属 or 購入） |
| hub_rr | A5052 | メカナムのハブアダプタ（ホイールに付属 or 購入） |
| lidar_high | SENSOR | LiDAR / カメラ（購入） |
| lidar_low_front | SENSOR | LiDAR / カメラ（購入） |
| lidar_low_rear | SENSOR | LiDAR / カメラ（購入） |
| lidar_lvl_spr_front0 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_front1 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_front2 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_front3 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_high0 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_high1 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_high2 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_high3 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_rear0 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_rear1 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_rear2 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lidar_lvl_spr_rear3 | STEEL | LiDAR レベリング座の圧縮ばね φ8×20（購入。BOM §3） |
| lift_belt_L | RUBBER | タイミングベルト（購入） |
| lift_belt_R | RUBBER | タイミングベルト（購入） |
| lift_belt_drv | RUBBER | タイミングベルト（購入） |
| lift_brg_hiL | A5052 | フランジ軸受ユニット（購入） |
| lift_brg_hiR | A5052 | フランジ軸受ユニット（購入） |
| lift_brg_loL | A5052 | フランジ軸受ユニット（購入） |
| lift_brg_loR | A5052 | フランジ軸受ユニット（購入） |
| lift_motor | MOTOR | DJI M3508 / M2006（購入） |
| lift_pul_L_hi | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| lift_pul_L_lo | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| lift_pul_R_hi | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| lift_pul_R_lo | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| lift_pul_d | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| lift_pul_m | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| lift_shaft_hiL | STEEL | φ5 鋼軸（切断のみ） |
| lift_shaft_hiR | STEEL | φ5 鋼軸（切断のみ） |
| lift_shaft_loL | STEEL | φ5 鋼軸（切断のみ） |
| lift_shaft_loR | STEEL | φ5 鋼軸（切断のみ） |
| mascot_badge | MASCOT_RAG | マスコットのゼッケン |
| mascot_bandana | MASCOT_TRIM | マスコットの三角巾 |
| mascot_eye_L | MASCOT_TRIM | マスコットの三角巾 |
| mascot_eye_R | MASCOT_TRIM | マスコットの三角巾 |
| mascot_foot_L | MASCOT_DARK | マスコットの靴・瞳 |
| mascot_foot_R | MASCOT_DARK | マスコットの靴・瞳 |
| mascot_head | MASCOT | マスコット（EPP + フェルト。規定 3.1.3 で重量制限外） |
| mascot_rag | MASCOT_RAG | マスコットが持つ雑巾 |
| mascot_suit | MASCOT_SUIT | マスコットのつなぎ |
| mast_arm_L | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| mast_arm_R | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| mast_cross | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| mcu | PCB | 制御基板（購入 or 別途設計） |
| motor_fl | MOTOR | DJI M3508 / M2006（購入） |
| motor_fr | MOTOR | DJI M3508 / M2006（購入） |
| motor_rl | MOTOR | DJI M3508 / M2006（購入） |
| motor_rr | MOTOR | DJI M3508 / M2006（購入） |
| nip_screw_L | STEEL | ニップ調整ねじ（購入。M8 全ねじ + ロックナット） |
| nip_screw_R | STEEL | ニップ調整ねじ（購入。M8 全ねじ + ロックナット） |
| nip_snut_L | A5052 | ニップ調整ねじの受けナット（購入。M8 フランジナット） |
| nip_snut_R | A5052 | ニップ調整ねじの受けナット（購入。M8 フランジナット） |
| odo_band00 | RUBBER | 予圧の輪ゴム #10（消耗品。試合ごとに掛け替える） |
| odo_band01 | RUBBER | 予圧の輪ゴム #10（消耗品。試合ごとに掛け替える） |
| odo_band10 | RUBBER | 予圧の輪ゴム #10（消耗品。試合ごとに掛け替える） |
| odo_band11 | RUBBER | 予圧の輪ゴム #10（消耗品。試合ごとに掛け替える） |
| odo_band20 | RUBBER | 予圧の輪ゴム #10（消耗品。試合ごとに掛け替える） |
| odo_band21 | RUBBER | 予圧の輪ゴム #10（消耗品。試合ごとに掛け替える） |
| odo_blk0 | STEEL | リニアガイド MGN9C ブロック（購入） |
| odo_blk1 | STEEL | リニアガイド MGN9C ブロック（購入） |
| odo_blk2 | STEEL | リニアガイド MGN9C ブロック（購入） |
| odo_boss0 | A5052 | 軸受ボス（購入 or PETG。φ14 の小物で切り抜きにならない） |
| odo_boss1 | A5052 | 軸受ボス（購入 or PETG。φ14 の小物で切り抜きにならない） |
| odo_boss2 | A5052 | 軸受ボス（購入 or PETG。φ14 の小物で切り抜きにならない） |
| odo_brg0i | STEEL | MR105ZZ 玉軸受（購入） |
| odo_brg0o | STEEL | MR105ZZ 玉軸受（購入） |
| odo_brg1i | STEEL | MR105ZZ 玉軸受（購入） |
| odo_brg1o | STEEL | MR105ZZ 玉軸受（購入） |
| odo_brg2i | STEEL | MR105ZZ 玉軸受（購入） |
| odo_brg2o | STEEL | MR105ZZ 玉軸受（購入） |
| odo_enc0 | PCB | 制御基板（購入 or 別途設計） |
| odo_enc1 | PCB | 制御基板（購入 or 別途設計） |
| odo_enc2 | PCB | 制御基板（購入 or 別途設計） |
| odo_mag0 | STEEL | エンコーダ用磁石（購入） |
| odo_mag1 | STEEL | エンコーダ用磁石（購入） |
| odo_mag2 | STEEL | エンコーダ用磁石（購入） |
| odo_rail0 | STEEL | リニアガイド MGN9 レール L64（購入） |
| odo_rail1 | STEEL | リニアガイド MGN9 レール L64（購入） |
| odo_rail2 | STEEL | リニアガイド MGN9 レール L64（購入） |
| odo_shaft0 | STEEL | φ5 鋼軸（切断のみ） |
| odo_shaft1 | STEEL | φ5 鋼軸（切断のみ） |
| odo_shaft2 | STEEL | φ5 鋼軸（切断のみ） |
| odo_wheel0 | URETHANE | 計測輪の双列オムニホイール φ50×W20（購入） |
| odo_wheel1 | URETHANE | 計測輪の双列オムニホイール φ50×W20（購入） |
| odo_wheel2 | URETHANE | 計測輪の双列オムニホイール φ50×W20（購入） |
| pc_dcdc | PCB | 制御基板（購入 or 別途設計） |
| pc_mini | PC | 制御 PC（購入） |
| pedestal_beam0 | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| pedestal_beam1 | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| pitch_motor | MOTOR | DJI M3508 / M2006（購入） |
| pitch_pivot_L | A5052 | 仰角軸受カラー 外径φ30×内径φ20×幅9（購入） |
| pitch_pivot_R | A5052 | 仰角軸受カラー 外径φ30×内径φ20×幅9（購入） |
| pitch_wbrg_m | STEEL | 6001ZZ 玉軸受（購入） |
| pitch_wbrg_p | STEEL | 6001ZZ 玉軸受（購入） |
| pitch_wbrk_m | A5052 | 軸受ホルダ 6001 用（購入。BOM §3「ウォーム軸まわり」） |
| pitch_wbrk_p | A5052 | 軸受ホルダ 6001 用（購入。BOM §3「ウォーム軸まわり」） |
| pitch_wcol_m | STEEL | スラストカラー（購入） |
| pitch_wcol_p | STEEL | スラストカラー（購入） |
| pitch_wcplg | A5052 | クランプ式軸継手 φ8-φ12（購入。BOM §3） |
| pitch_worm | STEEL | ウォーム m1.5 1条（購入） |
| pitch_worm_wheel | A5052 | ウォーム m1.5 1条（購入） |
| post_front_L | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| post_front_R | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| post_rear_L | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| post_rear_R | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| power_led | PCB | 電源表示 LED（購入） |
| press_bush_L | STEEL | フランジブッシュ（購入） |
| press_bush_R | STEEL | フランジブッシュ（購入） |
| press_clamp_L | STEEL | φ10 シャフトクランプ（購入） |
| press_clamp_R | STEEL | φ10 シャフトクランプ（購入） |
| press_guide_L | A5052 | φ10×φ6 アルミ中空軸（切断のみ。上押さえのガイド） |
| press_guide_R | A5052 | φ10×φ6 アルミ中空軸（切断のみ。上押さえのガイド） |
| press_motor | MOTOR | DJI M3508 / M2006（購入） |
| press_pad | SPONGE | 上押さえのスポンジパッド（シートから抜く） |
| rail_L_in | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| rail_L_out | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| rail_R_in | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| rail_R_out | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| rail_ball_L0d | STEEL | SRX3616 のボールリテーナ（購入品の一部） |
| rail_ball_L0u | STEEL | SRX3616 のボールリテーナ（購入品の一部） |
| rail_ball_L1d | STEEL | SRX3616 のボールリテーナ（購入品の一部） |
| rail_ball_L1u | STEEL | SRX3616 のボールリテーナ（購入品の一部） |
| rail_ball_R0d | STEEL | SRX3616 のボールリテーナ（購入品の一部） |
| rail_ball_R0u | STEEL | SRX3616 のボールリテーナ（購入品の一部） |
| rail_ball_R1d | STEEL | SRX3616 のボールリテーナ（購入品の一部） |
| rail_ball_R1u | STEEL | SRX3616 のボールリテーナ（購入品の一部） |
| rail_in_L | STEEL | MISUMI SRX3616 インナーレール（購入） |
| rail_in_R | STEEL | MISUMI SRX3616 インナーレール（購入） |
| rail_mid_L | STEEL | MISUMI SRX3616 中間レール（購入） |
| rail_mid_R | STEEL | MISUMI SRX3616 中間レール（購入） |
| rail_out_L | STEEL | MISUMI SRX3616 アウターレール（購入） |
| rail_out_R | STEEL | MISUMI SRX3616 アウターレール（購入） |
| ramp_belt_L | RUBBER | タイミングベルト（購入） |
| ramp_belt_R | RUBBER | タイミングベルト（購入） |
| ramp_pul_L_hi | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| ramp_pul_L_lo | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| ramp_pul_R_hi | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| ramp_pul_R_lo | A5052 | HTD5M タイミングプーリ（購入。BOM §3） |
| ramp_shaft_hi | STEEL | φ8 鋼軸（切断のみ） |
| ramp_shaft_lo | STEEL | φ8 鋼軸（切断のみ） |
| roller_belt_d | RUBBER | タイミングベルト（購入） |
| roller_belt_u | RUBBER | タイミングベルト（購入） |
| roller_brg_dL | STEEL | 深溝玉軸受（購入。BOM §3） |
| roller_brg_dR | STEEL | 深溝玉軸受（購入。BOM §3） |
| roller_brg_uL | STEEL | 深溝玉軸受（購入。BOM §3） |
| roller_brg_uR | STEEL | 深溝玉軸受（購入。BOM §3） |
| roller_d0 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| roller_d1 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| roller_d2 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| roller_d3 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| roller_d4 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| roller_motor_d | MOTOR | DJI M3508 / M2006（購入） |
| roller_motor_u | MOTOR | DJI M3508 / M2006（購入） |
| roller_pul_m_d | A5052 | HTD5M 24T タイミングプーリ（購入。BOM §3） |
| roller_pul_m_u | A5052 | HTD5M 24T タイミングプーリ（購入。BOM §3） |
| roller_pul_s_d | A5052 | HTD5M 24T タイミングプーリ（購入。BOM §3） |
| roller_pul_s_u | A5052 | HTD5M 24T タイミングプーリ（購入。BOM §3） |
| roller_shaft_d | A5052 | φ12 中空軸 A2017（切断のみ） |
| roller_shaft_u | A5052 | φ12 中空軸 A2017（切断のみ） |
| roller_u0 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| roller_u1 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| roller_u2 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| roller_u3 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| roller_u4 | URETHANE | 射出ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| scr_J1_fl_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_fl_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_fl_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_fl_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_fr_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_fr_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_fr_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_fr_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_rl_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_rl_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_rl_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_rl_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_rr_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_rr_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_rr_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J1_rr_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J2_fl_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J2_fl_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J2_fr_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J2_fr_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J2_rl_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J2_rl_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J2_rr_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J2_rr_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J5_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J5_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J5_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J5_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J5_4 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J5_5 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J5_6 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J5_7 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_L_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_L_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_L_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_L_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_L_4 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_L_5 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_R_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_R_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_R_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_R_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_R_4 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J6_R_5 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J8_L_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J8_L_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J8_R_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_J8_R_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0001 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0002 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0003 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0004 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0005 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0006 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0007 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0008 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0009 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0010 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0011 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0012 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0013 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0014 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0015 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0016 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0017 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0018 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0019 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0020 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0021 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0022 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0023 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0024 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0025 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0026 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0027 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0028 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0029 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0030 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0031 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0032 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0033 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0034 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0035 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0036 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0037 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0038 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0039 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0040 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0041 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0042 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0043 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0044 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0045 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0046 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0047 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0048 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0049 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0050 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0051 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0052 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0053 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0054 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0055 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0056 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0057 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0058 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0059 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0060 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0061 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0062 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0063 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0064 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0065 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0066 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0067 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0068 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0069 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0070 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0071 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0072 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0073 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0074 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0075 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0076 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0077 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0078 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0079 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0080 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0081 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0082 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0083 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0084 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0085 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0086 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0087 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0088 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0089 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0090 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0091 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0092 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0093 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0094 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0095 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0096 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0097 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0098 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0099 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0100 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0101 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0102 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0103 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0104 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0105 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0106 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0107 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0108 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0109 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0110 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0111 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0112 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0113 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0114 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0115 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0116 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0117 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0118 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0119 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0120 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0121 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0122 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0123 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0124 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0125 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0126 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0127 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0128 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0129 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0130 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0131 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0132 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0133 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0134 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0135 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0136 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0137 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0138 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0139 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0140 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0141 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0142 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0143 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0144 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0145 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0146 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0147 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0148 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0149 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0150 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0151 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0152 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0153 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0154 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0155 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0156 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0157 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0158 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0159 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0160 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0161 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0162 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0163 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0164 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0165 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0166 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0167 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0168 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0169 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0170 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0171 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0172 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0173 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0174 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0175 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0176 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0177 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0178 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0179 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0180 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0181 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0182 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0183 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0184 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0185 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0186 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0187 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0188 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0189 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0190 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0191 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0192 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0193 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0194 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0195 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0196 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0197 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0198 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0199 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0200 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0201 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0202 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0203 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0204 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0205 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0206 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0207 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0208 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0209 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0210 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0211 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0212 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0213 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0214 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0215 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0216 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0217 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0218 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0219 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0220 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0221 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0222 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0223 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0224 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0225 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0226 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0227 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0228 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0229 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0230 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0231 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0232 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0233 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0234 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0235 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0236 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0237 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0238 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0239 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0240 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0241 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0242 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0243 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0244 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0245 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0246 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0247 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0248 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0249 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0250 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0251 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0252 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0253 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0254 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0255 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0256 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0257 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0258 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0259 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0260 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0261 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0262 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0263 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0264 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0265 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0266 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0267 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0268 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0269 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0270 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0271 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0272 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0273 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0274 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0275 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0276 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0277 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0278 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0279 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0280 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0281 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0282 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0283 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0284 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0285 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0286 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0287 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0288 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0289 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0290 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0291 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0292 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0293 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0294 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0295 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0296 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0297 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0298 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0299 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0300 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0301 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0302 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0303 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0304 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0305 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0306 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0307 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0308 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0309 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0310 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0311 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0312 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0313 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0314 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0315 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0316 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0317 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0318 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0319 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0320 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0321 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0322 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0323 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0324 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0325 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0326 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0327 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0328 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0329 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0330 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0331 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0332 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0333 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0334 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0335 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0336 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0337 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0338 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0339 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0340 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0341 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0342 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0343 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0344 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0345 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0346 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0347 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0348 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0349 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0350 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0351 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0352 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0353 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0354 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0355 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0356 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0357 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0358 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0359 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0360 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0361 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0362 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0363 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0364 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0365 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0366 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0367 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0368 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0369 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0370 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0371 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0372 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0373 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0374 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0375 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0376 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0377 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0378 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0379 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0380 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0381 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0382 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0383 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0384 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0385 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0386 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0387 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0388 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0389 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0390 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_a0391 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bkseat_L0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bkseat_L1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bkseat_R0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bkseat_R1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bkt0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bkt1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bkt2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bkt3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bktab0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bktab1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bktab2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_bktab3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm110_mid_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm110_mid_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm110_mid_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm110_mid_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm210_mid_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm210_mid_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm210_mid_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm210_mid_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_L_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_L_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_L_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_L_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_R_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_R_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_R_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_R_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_mid_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_mid_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_mid_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xm410_mid_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp110_mid_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp110_mid_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp110_mid_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp110_mid_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp210_mid_mn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp210_mid_mn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp210_mid_pn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp210_mid_pn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_L_mn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_L_mn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_L_pn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_L_pn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_R_mn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_R_mn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_R_pn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_R_pn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_mid_mn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_mid_mn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_mid_pn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_cross_xp410_mid_pn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_disp_Ld_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_disp_Ld_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_disp_Lu_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_disp_Lu_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_disp_Rd_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_disp_Rd_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_disp_Ru_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_disp_Ru_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_arm_L_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_arm_L_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_arm_L_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_arm_L_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_arm_R_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_arm_R_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_arm_R_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_arm_R_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_corner_L_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_corner_L_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_corner_R_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_corner_R_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Lm_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Lm_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Lm_2 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Lm_3 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Lp_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Lp_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Lp_2 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Lp_3 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Rm_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Rm_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Rm_2 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Rm_3 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Rp_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Rp_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Rp_2 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_mast_cross_Rp_3 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_pedestal_beam0_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_pedestal_beam0_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_pedestal_beam0_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_pedestal_beam0_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_pedestal_beam1_mn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_pedestal_beam1_mn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_pedestal_beam1_pn_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_pedestal_beam1_pn_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_front_L_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_front_L_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_front_L_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_front_L_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_front_R_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_front_R_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_front_R_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_front_R_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_rear_L_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_rear_L_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_rear_L_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_rear_L_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_rear_R_m_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_rear_R_m_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_rear_R_p_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_post_rear_R_p_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam0_L_pd_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam0_L_pd_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam0_L_pu_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam0_L_pu_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam0_R_pd_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam0_R_pd_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam0_R_pu_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam0_R_pu_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_L_md_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_L_md_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_L_mu_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_L_mu_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_L_pd_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_L_pd_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_R_md_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_R_md_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_R_mu_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_R_mu_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_R_pd_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_brk_topbeam1_R_pd_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_car_rib_L_00 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_car_rib_L_10 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_car_rib_R_00 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_car_rib_R_10 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_disp_claw_d0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_disp_claw_d1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_disp_claw_u0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_disp_claw_u1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_grab_mot0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_grab_mot1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_grab_mot2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_grab_mot3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_L_hi_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_L_hi_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_L_hi_2 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_L_hi_3 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_L_lo_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_L_lo_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_L_lo_2 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_L_lo_3 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_R_hi_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_R_hi_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_R_hi_2 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_R_hi_3 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_R_lo_0 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_R_lo_1 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_R_lo_2 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_gus_brace_R_lo_3 | SUS304 | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_fl_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_fl_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_fl_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_fl_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_fr_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_fr_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_fr_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_fr_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_rl_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_rl_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_rl_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_rl_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_rr_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_rr_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_rr_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_hub_rr_3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_brg_hiL_00 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_brg_hiL_10 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_brg_hiR_00 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_brg_hiR_10 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_brg_loL_00 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_brg_loL_10 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_brg_loR_00 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_brg_loR_10 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_mot0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_mot1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_mot2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_mot3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_motbrk_00 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lift_motbrk_10 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_front0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_front1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_front2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_front3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_high0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_high1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_high2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_high3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_rear0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_rear1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_rear2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_lvl_rear3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_odo_hub0_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_odo_hub0_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_odo_hub0_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_odo_hub1_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_odo_hub1_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_odo_hub1_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_odo_hub2_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_odo_hub2_1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_odo_hub2_2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_pitch_mot0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_pitch_mot1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_pitch_mot2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_pitch_mot3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_press_bush_L_00 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_press_bush_L_10 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_press_bush_R_00 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_press_bush_R_10 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_press_mot0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_press_mot1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_press_mot2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_press_mot3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_mot_d0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_mot_d1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_mot_d2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_mot_d3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_mot_u0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_mot_u1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_mot_u2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_mot_u3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_stand_d0_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_stand_d1_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_stand_d2_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_stand_d3_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_stand_u0_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_stand_u1_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_stand_u2_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_roller_stand_u3_0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_sing_motbrk0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_sing_motbrk1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yaw_mot0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yaw_mot1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yaw_mot2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yaw_mot3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yawpul0 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yawpul1 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yawpul2 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yawpul3 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yawpul4 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yawpul5 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yawpul6 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| scr_yawpul7 | STEEL | ねじ・リベット（fasteners_bom.md の員数表で発注） |
| sing_brg_L | A5052 | フランジ軸受ユニット（購入） |
| sing_brg_R | A5052 | フランジ軸受ユニット（購入） |
| sing_bush_L | POM | POM フランジブッシュ（購入） |
| sing_bush_R | POM | POM フランジブッシュ（購入） |
| sing_cplg | A5052 | クランプ式軸継手 φ8-φ8（購入。BOM §3） |
| sing_motor | MOTOR | DJI M3508 / M2006（購入） |
| sing_pad0 | SILICONE | リタードパッド シリコン t3（シートから抜く消耗品） |
| sing_pad1 | SILICONE | リタードパッド シリコン t3（シートから抜く消耗品） |
| sing_pad2 | SILICONE | リタードパッド シリコン t3（シートから抜く消耗品） |
| sing_pad3 | SILICONE | リタードパッド シリコン t3（シートから抜く消耗品） |
| sing_pad4 | SILICONE | リタードパッド シリコン t3（シートから抜く消耗品） |
| sing_rotor | MOTOR_SHAFT | モーターの出力軸（購入品の一部） |
| sing_shaft | A5052 | φ8 中空軸（切断のみ） |
| sing_tire0 | URETHANE | 分離ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| sing_tire1 | URETHANE | 分離ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| sing_tire2 | URETHANE | 分離ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| sing_tire3 | URETHANE | 分離ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| sing_tire4 | URETHANE | 分離ローラーのウレタンタイヤ t3（シートから抜く消耗品） |
| thk_hall_L | PCB | 制御基板（購入 or 別途設計） |
| thk_hall_R | PCB | 制御基板（購入 or 別途設計） |
| topbeam0_L | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| topbeam0_R | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| topbeam1_L | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| topbeam1_R | A6005C | MISUMI HFS5-2020（指定長カット。BOM.md §1 の切断リスト） |
| wheel_fl | URETHANE | メカナムホイール φ100（購入。BOM §3） |
| wheel_fr | URETHANE | メカナムホイール φ100（購入。BOM §3） |
| wheel_rl | URETHANE | メカナムホイール φ100（購入。BOM §3） |
| wheel_rr | URETHANE | メカナムホイール φ100（購入。BOM §3） |
| yaw_ext_shaft | STEEL | φ18 延長軸（切断のみ） |
| yaw_motor | MOTOR | DJI M3508 / M2006（購入） |
| yaw_pulley_big | A5052 | HTD5M タイミングプーリ 20T/60T（購入。BOM §3） |
| yaw_pulley_small | A5052 | HTD5M タイミングプーリ 20T/60T（購入。BOM §3） |
| yaw_ring | STEEL | 旋回 V リング φ220（購入。BOM §3） |

## 手が要るもの 2 件

- `disp_screen` … 作れない: SCREEN は板材として買えない（板厚 1.0mm）
- `lift_mot_brk` … 作れない: 平板でない。**3D の削り出しはできない**
