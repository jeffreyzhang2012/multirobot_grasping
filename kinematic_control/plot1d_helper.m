function plot1d_helper(t, data, item, item_label)
    % data shape: [ndim, horizon]
    ndim = size(data, 1);
    hold on
    for dim = 1 : ndim
        subplot(ndim, 1, dim)
        plot(t, data(dim, :))
        ylabel(append(item_label, '_', num2str(dim, '%i')))
        grid on
        xlabel('t')
    end
    subplot(ndim, 1, 1)
    title(item)
    hold off
end