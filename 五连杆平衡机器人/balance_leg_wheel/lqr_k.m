function K = lqr_k(L0)
%LQR_K
%    K = LQR_K(L0)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2024-07-31 20:42:29

t2 = L0.^2;
t3 = L0.^3;
mt1 = [L0.*(-8.065271772558496e+1)-t2.*1.205868006840707e+2+t3.*1.707869922428602e+2-2.044918044897489e+1];
mt2 = [L0.*5.665894760840532e+1-t2.*1.827041217760809e+2+t3.*1.595744504812248e+2+3.60669465642944e+1];
mt3 = [L0.*(-6.507625934256153)-t2.*6.514107071439187e+1+t3.*5.487972990029279e+1-2.840764191045059];
mt4 = [L0.*2.205303064584527e+1-t2.*3.201196102597935e+1+t3.*1.854853133140874e+1+3.468032294427596];
mt5 = [L0.*2.160323102967916e-2-t2.*2.185279381850989+t3.*3.092574439904803-5.059693352764429e-1];
mt6 = [L0.*(-2.924656864403308)+t2.*3.373905873942121-t3.*1.560765787499464+1.301208122422391];
mt7 = [L0.*9.093526032554543e-1-t2.*5.145541645210415e+1+t3.*7.193321556731189e+1-1.157850409516822e+1];
mt8 = [L0.*(-6.658338719144126e+1)+t2.*7.788804225295389e+1-t3.*3.706464042217822e+1+2.955041484989111e+1];
mt9 = [L0.*(-6.662787839379628e+1)+t2.*4.519441613445697e+1+t3.*5.463691176747377+3.74351621982198e+1];
mt10 = [L0.*1.523057731625358e+2-t2.*2.547075927329871e+2+t3.*1.622299028545877e+2+4.386419361563708e+1];
mt11 = [L0.*(-1.17424870998693e+1)+t2.*2.230983138349281e+1-t3.*1.867584514756801e+1+4.671235904494449];
mt12 = [L0.*2.028626432988079e+1-t2.*3.738273771912743e+1+t3.*2.751377335350262e+1-5.204303323246763e-3];
K = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12],2,6);
end
