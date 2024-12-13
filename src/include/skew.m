function skew_h = skew(h)
    skew_h = [
        0, -h(3), h(2);
        h(3), 0, -h(1);
        -h(2), h(1), 0;
    ];
end