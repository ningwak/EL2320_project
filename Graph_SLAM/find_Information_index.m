function index = find_information_index(x, list, type)

    mapAdd = 3 * size(x, 2);
    index = ([3 * list' - 2, 3 * list' - 1, 3 * list'])';
    index = reshape(index, 1, []);
    if (type == 'map')
        index = index + mapAdd;
    end
end