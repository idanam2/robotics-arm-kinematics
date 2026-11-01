function robot_gui_ellipse()
    clear; clc; close all;

    %% 1. טעינת קבצים
    try
        TR0 = stlread('models/base.STL');
        TR1 = stlread('models/link1.STL');
        TR2 = stlread('models/link1.STL');
        TR3 = stlread('models/link2.STL');
        TR4 = stlread('models/endlink3.STL');
        TR5 = stlread('models/tippen.STL');
    catch
        [x,y,z] = cylinder(5, 10);
        TR0 = triangulation(convhull(x(:),y(:),z(:)), [x(:) y(:) z(:)]);
        TR1 = TR0; TR2 = TR0; TR3 = TR0; TR4 = TR0; TR5 = TR0;
    end

    pts0 = TR0.Points; pts1 = TR1.Points; pts2 = TR2.Points;
    pts3 = TR3.Points; pts4 = TR4.Points; pts5 = TR5.Points;

    %% 2. הגדרת הגרף
    f = figure('Name', 'Robot DH - Fixed Kinematics', 'Color', 'w');
    axis equal; grid on; view(135, 30); hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-400 600]); ylim([-400 600]); zlim([-100 600]);

    % PATCHES
    hBase  = patch('Faces', TR0.ConnectivityList, 'Vertices', pts0, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
    hLink1 = patch('Faces', TR1.ConnectivityList, 'Vertices', pts1, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
    hLink2 = patch('Faces', TR2.ConnectivityList, 'Vertices', pts2, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
    hLink3 = patch('Faces', TR3.ConnectivityList, 'Vertices', pts3, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
    hLink4 = patch('Faces', TR4.ConnectivityList, 'Vertices', pts4, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
    hLink5 = patch('Faces', TR5.ConnectivityList, 'Vertices', pts5, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');

    light; lighting gouraud;

    % Frames
    hFrame0 = draw_frame(eye(4), 60); hFrame1 = draw_frame(eye(4), 60);
    hFrame2 = draw_frame(eye(4), 60); hFrame3 = draw_frame(eye(4), 60);
    hFrame4 = draw_frame(eye(4), 60); hFrame5 = draw_frame(eye(4), 60);

    % מסלול אדום
    hPath = plot3(nan,nan,nan,'r-','LineWidth',2);
    hTip = plot3(0,0,0,'ko','MarkerSize',7,'MarkerFaceColor','k');

    %% 3. סליידרים
    sld1 = uicontrol('Style','slider','Min',-180,'Max',180,'Value',0,'Units','normalized','Position',[0.1 0.1 0.15 0.05],'Callback',@update_robot);
    sld2 = uicontrol('Style','slider','Min',-90,'Max',90,'Value',0,'Units','normalized','Position',[0.3 0.1 0.15 0.05],'Callback',@update_robot);
    sld3 = uicontrol('Style','slider','Min',-90,'Max',90,'Value',0,'Units','normalized','Position',[0.5 0.1 0.15 0.05],'Callback',@update_robot);
    sld4 = uicontrol('Style','slider','Min',-90,'Max',90,'Value',0,'Units','normalized','Position',[0.7 0.1 0.15 0.05],'Callback',@update_robot);
    sld5 = uicontrol('Style','slider','Min',-90,'Max',90,'Value',0,'Units','normalized','Position',[0.1 0.02 0.15 0.05],'Callback',@update_robot);
    sld6 = uicontrol('Style','slider','Min',-180,'Max',180,'Value',0,'Units','normalized','Position',[0.3 0.02 0.15 0.05],'Callback',@update_robot);

    uicontrol('Style','pushbutton','String','Animate Red Ellipse', ...
        'Units','normalized','Position',[0.6 0.02 0.2 0.06], ...
        'Callback',@animate_ellipse_tip);

    update_robot();

    %% 4. ליבת ה-DH המקורית (ללא שינוי)
    function update_robot(~,~)
        th1 = sld1.Value; th2 = sld2.Value; th3 = sld3.Value;
        th4 = sld4.Value; th5 = sld5.Value; th6 = sld6.Value;
        toolLen = 100;

        T_01 = dh(th1,   0,      0,   0 );
        T_12 = dh(th2,   0,      0,  90 );
        T_23 = dh(th3,   0,    125,   0 );
        T_34 = dh(th4,   0,    125,   0 );
        T_45 = dh(th5, 37.5,    0,  90 );
        T_56_base = dh(0, 104,  0,   0 );

        T_Base_K   = T_01;
        T_Link2_K  = T_Base_K * T_12 * rotz_mat(90);
        T_Link3_K  = T_Link2_K * T_23;
        T_Link4_K  = T_Link3_K * T_34 * rotz_mat(90);
        T_Link5_K  = T_Link4_K * T_45 * rotz_mat(90);
        T_Link6_K  = T_Link5_K * T_56_base * rotx_mat(90) * roty_mat(90);

        T_Link6_Draw = T_Link6_K * rotz_mat(th6);
        T_Tool = transl_mat(0, toolLen, 0); 
        T_Tip  = T_Link6_Draw * T_Tool

        set(hBase,  'Vertices', apply_mat(T_Base_K,  pts0));
        set(hLink1, 'Vertices', apply_mat(T_Link2_K, pts1));
        set(hLink2, 'Vertices', apply_mat(T_Link3_K, pts2));
        set(hLink3, 'Vertices', apply_mat(T_Link4_K, pts3));
        set(hLink4, 'Vertices', apply_mat(T_Link5_K, pts4));
        set(hLink5, 'Vertices', apply_mat(T_Link6_Draw, pts5));

        update_frame(hFrame0, T_Base_K); update_frame(hFrame1, T_Link2_K);
        update_frame(hFrame2, T_Link3_K); update_frame(hFrame3, T_Link4_K);
        update_frame(hFrame4, T_Link5_K); update_frame(hFrame5, T_Link6_Draw);

        pTip = T_Tip(1:3,4);
        set(hTip,'XData',pTip(1),'YData',pTip(2),'ZData',pTip(3));
        drawnow limitrate;
    end

    %% 5. פונקציית עזר לחישוב TIP (משתמשת באותה לוגיקה בדיוק)
    function p = get_tip_pos(t1, t2, t3, t4, t5, t6)
        toolLen = 100;
        T_01 = dh(t1, 0, 0, 0);
        T_12 = dh(t2, 0, 0, 90);
        T_23 = T_01 * T_12 * rotz_mat(90) * dh(t3, 0, 125, 0);
        T_34 = T_23 * dh(t4, 0, 125, 0) * rotz_mat(90);
        T_45 = T_34 * dh(t5, 37.5, 0, 90) * rotz_mat(90);
        T_L6 = T_45 * dh(0, 104, 0, 0) * rotx_mat(90) * roty_mat(90) * rotz_mat(t6);
        T_Tip = T_L6 * transl_mat(0, toolLen, 0);
        p = T_Tip(1:3,4);
    end

   %% 6. אנימציה לפי דרישות השאלה (101 נקודות, אליפסה מוטה, 8 טבלאות)
   function animate_ellipse_tip(~,~)
        % 1. הגדרות המשימה
        a_ell = 150; b_ell = 100; N = 101;
        alpha = 25; beta = 15; gamma = 10;
        R_plane = rotz_mat(alpha) * roty_mat(beta) * rotx_mat(gamma);
        center = [180; 0; 300];
        sld5.Value = 0;
        sld6.Value = 0;
        
        % 2. 8 נקודות התחלה ל-8 פתרונות
        guesses = [
            0,   30,  30,  0;   
            0,   60, -60,  0;   
            0,   30,  30,  180; 
            0,   60, -60,  180; 
            180, -30, -30,  0;   
            180, -60,  60,  0;   
            180, -30, -30,  180; 
            180, -60,  60,  180; 
        ];

        for s = 1:8
            % איפוס נתונים לכל פתרון
            set(hPath,'XData',[],'YData',[],'ZData',[]);
            Xtrail = []; Ytrail = []; Ztrail = [];
            table_data = zeros(N, 9);
            
            % אתחול ניחוש ראשוני מהטבלה
            th1_offset = guesses(s,1);
            cur_guess = guesses(s,2:4);
            
            fprintf('מחשב טבלה %d מתוך 8...\n', s);
            
            for k = 1:N
                % חישוב הנקודה על האליפסה
                phi = 2*pi*(k-1)/(N-1);
                p_local = [a_ell*cos(phi); b_ell*sin(phi); 0];
                target = center + R_plane(1:3,1:3) * p_local;
                
                % --- הגדרת th1 (כאן נפתרת השגיאה שלך) ---
                th1 = atan2d(target(2), target(1)) + th1_offset;
                
                % IK נומרי
                opts = optimset('Display','off','MaxIter',50);
                cost = @(x) norm(get_tip_pos(th1, x(1), x(2), x(3), sld5.Value, sld6.Value) - target)^2;
                cur_guess = fminsearch(cost, cur_guess, opts);
                
                % עדכון סליידרים ורובוט
                sld1.Value = th1;
                sld2.Value = cur_guess(1); 
                sld3.Value = cur_guess(2); 
                sld4.Value = cur_guess(3);
                update_robot(); 
                
                % עדכון הקו האדום
                Xtrail(end+1) = target(1); 
                Ytrail(end+1) = target(2); 
                Ztrail(end+1) = target(3);
                set(hPath, 'XData', Xtrail, 'YData', Ytrail, 'ZData', Ztrail);
                
                % שמירת נתונים לטבלה
                table_data(k, :) = [target', th1, cur_guess, sld5.Value, sld6.Value];
                
                % --- שליטה במהירות האנימציה ---
                drawnow; 
                pause(0.002); % 0.05 שניות להאטה. לשינוי המהירות שנה כאן.
            end
            
            % שמירת הטבלה ל-Workspace
            T = table(table_data(:,1), table_data(:,2), table_data(:,3), table_data(:,4:9), ...
                'VariableNames', {'X','Y','Z','Joint_Angles'});
            assignin('base', ['Solution_Table_' num2str(s)], T);
        end
    end

    %% פונקציות מתמטיות (ללא שינוי)
    function T = dh(theta, d, a, alpha)
        ct = cosd(theta); st = sind(theta); ca = cosd(alpha); sa = sind(alpha);
        T = [ct -st 0 a; st*ca ct*ca -sa -d*sa; st*sa ct*sa ca d*ca; 0 0 0 1];
    end
    function pts_out = apply_mat(T, pts_in)
        P = [pts_in, ones(size(pts_in,1),1)]';
        P_new = T * P; pts_out = P_new(1:3, :)';
    end
    function T = transl_mat(dx,dy,dz)
        T = eye(4); T(1:3,4) = [dx;dy;dz];
    end
    function R = rotz_mat(deg)
        c = cosd(deg); s = sind(deg);
        R = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
    end
    function R = roty_mat(deg)
        c = cosd(deg); s = sind(deg);
        R = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
    end
    function R = rotx_mat(deg)
        c = cosd(deg); s = sind(deg);
        R = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
    end
    function h = draw_frame(T, len)
        O = T(1:3,4);
        h.x = plot3([O(1) O(1)+T(1,1)*len], [O(2) O(2)+T(2,1)*len], [O(3) O(3)+T(3,1)*len], 'r', 'LineWidth', 2);
        h.y = plot3([O(1) O(1)+T(1,2)*len], [O(2) O(2)+T(2,2)*len], [O(3) O(3)+T(3,2)*len], 'g', 'LineWidth', 2);
        h.z = plot3([O(1) O(1)+T(1,3)*len], [O(2) O(2)+T(2,3)*len], [O(3) O(3)+T(3,3)*len], 'b', 'LineWidth', 2);
    end
    function update_frame(h, T)
        len = 50; O = T(1:3,4);
        set(h.x, 'XData', [O(1) O(1)+T(1,1)*len], 'YData', [O(2) O(2)+T(2,1)*len], 'ZData', [O(3) O(3)+T(3,1)*len]);
        set(h.y, 'XData', [O(1) O(1)+T(1,2)*len], 'YData', [O(2) O(2)+T(2,2)*len], 'ZData', [O(3) O(3)+T(3,2)*len]);
        set(h.z, 'XData', [O(1) O(1)+T(1,3)*len], 'YData', [O(2) O(2)+T(2,3)*len], 'ZData', [O(3) O(3)+T(3,3)*len]);
    end
end