#!/usr/bin/env python3
"""Small deterministic input-contract tests, independent of ROS scheduling."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
sys.path.insert(0,str(Path(__file__).parent/'scripts'))
from input_contract import HeadingCache

def heading(t=1_000_000_000, seq=1, **kw):
    m=S(header=S(stamp=S(sec=t//10**9,nanosec=t%10**9),frame_id='sim_gravity_heading'),
        yaw_rad=0.,yaw_rate_rad_s=.4,sample_sequence=seq,mcu_time_ms=0,
        source_boot_id=1,reset_counter=0,valid=True,online=True)
    for k,v in kw.items():setattr(m,k,v)
    return m

class ContractTests(unittest.TestCase):
    def setUp(self):self.c=HeadingCache('sim_gravity_heading')
    def test_both_async_orders_and_deadline(self):
        for arrival,raw in [(1.,1.02),(1.02,1.)]:
            self.c.add(heading(),arrival)
            self.assertIsNotNone(self.c.match(10**9,raw,1.03)[0])
            self.c=HeadingCache('sim_gravity_heading')
        self.c.add(heading(),1.)
        self.assertEqual(self.c.match(10**9,1.061,1.07)[1][0],'INVALID_TIME')
    def test_exact_duplicate_never_refreshes(self):
        self.c.add(heading(),1.);self.c.add(heading(),1.08)
        self.assertEqual(self.c.entries[10**9].arrived,1.)
        self.assertEqual(self.c.match(10**9,1.08,1.09)[1][0],'INVALID_TIME')
    def test_new_stamp_same_sequence_is_detectable(self):
        self.c.add(heading(),1.);self.c.add(heading(1_020_000_000),1.02)
        self.assertFalse(self.c.entries);self.assertEqual(self.c.problem[0],'INVALID_TIME')
        self.c.add(heading(1_040_000_000,2),1.04)
        self.assertIsNone(self.c.problem)
    def test_heading_stream_reordering_and_old_sequence_contradiction(self):
        self.c.add(heading(seq=2),1.)
        self.c.add(heading(980_000_000,1),1.01)
        self.assertIsNotNone(self.c.match(980_000_000,1.,1.02)[0])
        self.assertEqual(self.c.latest.message.sample_sequence,2)
        self.c.add(heading(1_020_000_000,1),1.02)
        self.assertFalse(self.c.entries)
    def test_historical_duplicate_does_not_refresh_or_change(self):
        self.c.add(heading(),1.);self.c.add(heading(1_020_000_000,2),1.02)
        self.c.add(heading(),1.08)
        self.assertEqual(self.c.entries[10**9].arrived,1.)
        self.c.add(heading(990_000_000,1),1.09)
        self.assertFalse(self.c.entries);self.assertEqual(self.c.problem[0],'INVALID_TIME')
    def test_sequence_wrap_and_cache_bound(self):
        self.c.add(heading(seq=0xffffffff),1.)
        self.c.add(heading(1_020_000_000,0),1.02)
        self.assertIsNone(self.c.problem)
        for i in range(1,105):self.c.add(heading(1_020_000_000+i*20_000_000,i),1.02+i*.02)
        self.assertEqual(len(self.c.entries),100)
    def test_offline_clears_and_duplicate_cannot_restore(self):
        self.c.add(heading(),1.);self.c.add(heading(1_020_000_000,2,online=False),1.02)
        self.c.add(heading(),1.03)
        self.assertFalse(self.c.entries);self.assertEqual(self.c.problem[0],'INVALID_INPUT')
        self.c.add(heading(1_040_000_000,3),1.04)
        self.assertIsNone(self.c.problem)
    def test_invalid_reference_and_rate(self):
        for kind in ['frame','rate']:
            m=heading();m.header.frame_id='wrong' if kind=='frame' else 'sim_gravity_heading'
            if kind=='rate':m.yaw_rate_rad_s=float('nan')
            self.c.add(m,1.);self.assertEqual(self.c.problem[0],'INVALID_INPUT')
    def test_epoch_observed_before_validity_and_matching(self):
        m=heading();self.c.add(m,1.);self.c.bind(m)
        self.c.add(heading(2_000_000_000,2,reset_counter=1,valid=False),1.02)
        self.c.add(heading(1_020_000_000,3),1.03)
        self.assertTrue(self.c.latched);self.assertFalse(self.c.entries)
    def test_wait_and_skew_preserved(self):
        self.c.add(heading(),1.)
        self.assertEqual(self.c.match(1_020_000_000,1.01,1.02),(None,None))
        m,problem=self.c.match(1_020_000_000,1.01,1.08)
        self.assertEqual(m.header.stamp.nanosec,0);self.assertIsNone(problem)
    def test_hidden_delay_is_not_observable(self):
        self.c.add(heading(),1.)
        self.c.add(heading(1_020_000_000,2,yaw_rad=-.008),1.02)
        self.assertIsNotNone(self.c.match(1_020_000_000,1.02,1.03)[0])

if __name__=='__main__':unittest.main(verbosity=2)
