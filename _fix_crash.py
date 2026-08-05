p = 'cli/tests/crash_suite.rs'
s = open(p, encoding='utf-8').read()
s = s.replace('theta_star_smooth(&p, &check).len(), 0', 'theta_star_smooth(&p, &check, None).len(), 0')
s = s.replace('theta_star_smooth(&p, &check).len(), 1', 'theta_star_smooth(&p, &check, None).len(), 1')
s = s.replace('let _ = theta_star_smooth(&bad, &check);', 'let _ = theta_star_smooth(&bad, &check, None);')
s = s.replace('let _ = theta_star_smooth(&p, &check);', 'let _ = theta_star_smooth(&p, &check, None);')
s = s.replace('ThetaStarSmoother { check: &check }', 'ThetaStarSmoother { check: &check, max_turn_deg: None }')
open(p, 'w', encoding='utf-8').write(s)
print('done')
