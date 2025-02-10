
### custom_goals.py
```
rom_interfaces::srv::ConstructYaml ကို သုံးပြီး SERVICE server တစ်ခုဖန်တီးပါ။

အဲ့ဒီ ထဲက waypoints များကို waypoinsts.yaml မှာ ရှိမရှိ စစ်ဆေးပြီး ရှိရင် Navigate Through Pose action client ဖန်တီးပါ။

navigation_topic လာရင် cancel လုပ်ပြီး break လုပ်ပါ။
```
### custom_goals_loop.py
```
အဲ့ဒီ ၃ ခု အဆင်ပြေသွားရင် loop ပါတဲ့ version နဲ့ မပါတဲ့ version ရေးပါ။

```

### အဲ့ဒီ loop ၄ ခုလုံးကို run ပေးနိုင်မယ့် calle script ကို ဘယ်မှာရေးမလဲ စဥ်းစားပါ။ script များသည် navigate_stop ပေးတာနဲ့ shutdown ဖြစ်နိုင်သလားစဥ်းစားပါ။
### shutdown ဖြစ်ရင် process id ကို ကိုင်ဖို့ မလိုတော့ဘူးပေါ့။