
function [rr, cc] = bresenham(y0, x0, y1, x1)
    rr = [];
    cc = [];
    steep = abs(y1 - y0) > abs(x1 - x0);

    if steep
        [x0, y0] = deal(y0, x0);
        [x1, y1] = deal(y1, x1);
    end
    if x0 > x1
        [x0, x1] = deal(x1, x0);
        [y0, y1] = deal(y1, y0);
    end

    dx = x1 - x0;
    dy = abs(y1 - y0);
    error = dx / 2;
    y = y0;
    ystep = 1;
    if y0 > y1
        ystep = -1;
    end

    for x = x0:x1
        if steep
            rr = [rr; x];
            cc = [cc; y];
        else
            rr = [rr; y];
            cc = [cc; x];
        end
        error = error - dy;
        if error < 0
            y = y + ystep;
            error = error + dx;
        end
    end
end