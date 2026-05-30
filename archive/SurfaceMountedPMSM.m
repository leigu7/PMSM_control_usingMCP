classdef SurfaceMountedPMSM
    % SurfaceMountedPMSM  Surface-mounted PMSM with table-based Ld/Lq/flux and temperature-dependent Rs.
    %
    %   This class provides 2-D interpolation for Ld, Lq, flux_d, flux_q
    %   as a function of Id and Iq, and 1-D interpolation for Rs as a
    %   function of temperature.
    
    properties
        IdGrid double
        IqGrid double
        LdMap double
        LqMap double
        psiDMap double
        psiQMap double
        TempGrid double
        RsMap double
        PolePairs double = 4
        J double = 0.005
        B double = 1e-3
    end
    methods
        function obj = SurfaceMountedPMSM()
            % Build default lookup tables for a surface-mounted PMSM.
            obj.IdGrid = linspace(-150, 150, 31);
            obj.IqGrid = linspace(-150, 150, 31);
            [Id, Iq] = meshgrid(obj.IdGrid, obj.IqGrid);

            baseLd = 1e-3;
            baseLq = 1e-3;
            obj.LdMap = baseLd .* (1 + 0.02 .* sin(0.012 .* Id) .* cos(0.012 .* Iq));
            obj.LqMap = baseLq .* (1 + 0.02 .* cos(0.012 .* Id) .* sin(0.012 .* Iq));

            fluxBase = 0.08;
            obj.psiDMap = fluxBase + 0.0002 .* Id + 0.00008 .* Iq;
            obj.psiQMap = fluxBase + 0.00008 .* Id + 0.0002 .* Iq;

            obj.TempGrid = [0, 25, 100];
            obj.RsMap = [2.8, 3.0, 3.4];
        end

        function Rs = interpRs(obj, temperature)
            % interpRs  Interpolate stator resistance versus temperature.
            Rs = interp1(obj.TempGrid, obj.RsMap, temperature, 'linear', 'extrap');
        end

        function [Ld, Lq, psi_d, psi_q] = lookup(obj, Id, Iq)
            % lookup  Return interpolated Ld, Lq, psi_d, psi_q for current state.
            Ld = interp2(obj.IdGrid, obj.IqGrid, obj.LdMap, Id, Iq, 'linear', obj.LdMap(1,1));
            Lq = interp2(obj.IdGrid, obj.IqGrid, obj.LqMap, Id, Iq, 'linear', obj.LqMap(1,1));
            psi_d = interp2(obj.IdGrid, obj.IqGrid, obj.psiDMap, Id, Iq, 'linear', obj.psiDMap(1,1));
            psi_q = interp2(obj.IdGrid, obj.IqGrid, obj.psiQMap, Id, Iq, 'linear', obj.psiQMap(1,1));
        end

        function Te = torque(obj, Id, Iq)
            % torque  Calculate electromagnetic torque on the motor.
            [Ld, Lq, psi_d, psi_q] = obj.lookup(Id, Iq);
            Te = 1.5 * obj.PolePairs * (psi_q .* Iq + (Ld - Lq) .* Id .* Iq);
        end
    end
end
