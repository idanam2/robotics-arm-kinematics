function inverse_kinematics_8sol()
    clc;
    fprintf('--- Robot IK Solver (8 Solutions - Analytical) ---\n');
    
    % 1. קלט מהמשתמש
    px = input('Enter Target X: ');
    py = input('Enter Target Y: ');
    pz = input('Enter Target Z: ');
    P_target = [px; py; pz];
    
    % 2. הגדרת אוריינטציה - עט ניצב לדף (Z עולמי)
    R_target = [1, 0,  0; 
                0, 0,  1; 
                0, -1, 0]; 
                
    % 3. נתוני רובוט
    L2 = 125; L3 = 125; toolOff = 37.5; 
    penLen = 100; 
    
    % 4. מציאת מיקום ה-Wrist Center
    P_j6 = P_target - R_target * [0; penLen; 0];
    xc = P_j6(1); yc = P_j6(2); zc = P_j6(3);
    
    % בדיקת טווח
    r_xy_raw = sqrt(xc^2 + yc^2);
    r_for_reach = r_xy_raw - toolOff;
    D = sqrt(r_for_reach^2 + zc^2);
    
    if D > (L2 + L3) || D < abs(L2 - L3)
        fprintf('\nRESULT: Point out of reach!\n');
        return;
    end

    solutions = [];
    % אפשרויות בסיס (2 פתרונות)
    th1_opts = [atan2d(yc, xc), atan2d(yc, xc) + 180];
    
    for th1 = th1_opts
        % חישוב המרחק האופקי מהציר השני (תלוי ב-th1)
        r = sqrt(xc^2 + yc^2) * cosd(th1 - atan2d(yc, xc)) - toolOff;
        
        cosTh3 = (r^2 + zc^2 - L2^2 - L3^2) / (2 * L2 * L3);
        if abs(cosTh3) > 1, continue; end
        
        % אפשרויות מרפק (2 פתרונות: Up/Down)
        th3_opts = [acosd(cosTh3), -acosd(cosTh3)];
        
        for th3 = th3_opts
            alpha = atan2d(zc, r);
            % משפט הקוסינוסים לזווית th2
            cos_beta = (L2^2 + (r^2 + zc^2) - L3^2) / (2 * L2 * sqrt(r^2 + zc^2));
            beta = acosd(max(min(cos_beta,1),-1));
            
            % תלוי בסימן של th3, ה-beta משנה כיוון
            if th3 > 0
                th2 = (alpha - beta) - 90;
            else
                th2 = (alpha + beta) - 90;
            end
            
            % --- חישוב כף יד (Wrist) להגעה ל-8 פתרונות ---
            % זווית הגישה הנדרשת כדי שהעט יהיה אנכי:
            pitch_required = -(th2 + th3 + 90);
            
            % פתרון כף יד 1: רגיל
            th4_1 = 0;
            th5_1 = pitch_required;
            th6_1 = 0;
            
            % פתרון כף יד 2: היפוך (Wrist Flip)
            % מסובבים את "הזרוע" של הכלי ב-180 מעלות והופכים את המפרק ה-5
            th4_2 = 180;
            th5_2 = -pitch_required;
            th6_2 = 180;
            
            solutions = [solutions; th1, th2, th3, th4_1, th5_1, th6_1];
            solutions = [solutions; th1, th2, th3, th4_2, th5_2, th6_2];
        end
    end
    
    % הצגת התוצאות
    if isempty(solutions)
        disp('No solutions found.');
    else
        fprintf('\nSUCCESS! 8 solutions generated:\n');
        T = table(solutions(:,1), solutions(:,2), solutions(:,3), ...
                  solutions(:,4), solutions(:,5), solutions(:,6), ...
                  'VariableNames', {'Th1_Base','Th2_Shoulder','Th3_Elbow','Th4_WristRoll','Th5_WristPitch','Th6_PenRoll'});
        disp(T);
        fprintf('Note: Solutions 1-4 are Standard, 5-8 are with Wrist Flip (180deg).\n');
    end
end