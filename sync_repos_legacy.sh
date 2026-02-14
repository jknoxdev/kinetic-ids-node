#!/bin/bash
echo '--- notes    ---'
cd ~/code/jknoxdev/notes
git pull
echo '--- scripts  ---'
cd ~/code/jknoxdev/scripts
git pull
echo '--- sites	   ---'
cd ~/code/jknoxdev/sites
git pull
echo '--- n3s      ---'
cd ~/code/jknoxdev/n3s
git pull
echo '--- videolan ---'
cd ~/code/jknoxdev/video-lan-checker
git pull
echo '--- jknoxdev ---'
cd ~/code/jknoxdev/jknoxdev
git pull
echo '--- blog     ---'
cd ~/code/jknoxdev/blog
git pull
echo '--- courswrk ---'
cd ~/code/jknoxdev/coursework
git pull
echo '--- hack     ---'
cd ~/code/jknoxdev/hack
git pull
echo '--- toosl    ---'
cd ~/code/jknoxdev/tools
git pull
echo '--- ctf-toos ---'
cd ~/code/jknoxdev/ctf-tools
git pull
echo '--- ghp-site ---'
cd ~/code/jknoxdev/jknoxdev.github.io
git pull
echo '--- ghp-site ---'
cd ~/code/jknoxdev/www.b3x9.github.io
git pull
echo '--- ghp-site ---'
cd ~/code/jknoxdev/shop.b3x9.github.io
git pull
echo '--- ghp-site ---'
cd ~/code/jknoxdev/blog.b3x9.github.io
git pull
echo '--- ghp-site ---'
cd ~/code/jknoxdev/jknox.b3x9.github.io
git pull
echo '--- pids-tlm ---'
cd ~/code/jknoxdev/physical-ids-telemetry.io
git pull

echo 'done!'

echo '--- notes    ---'
cd ~/code/jknoxdev/notes
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- scripts  ---'
cd ~/code/jknoxdev/scripts
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- sites    ---'
cd ~/code/jknoxdev/sites
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- n3s      ---'
cd ~/code/jknoxdev/n3s
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- videolan ---'
cd ~/code/jknoxdev/video-lan-checker
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- jknoxdev ---'
cd ~/code/jknoxdev/jknoxdev
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- blog     ---'
cd ~/code/jknoxdev/blog
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- courswrk ---'
cd ~/code/jknoxdev/coursework
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- hack     ---'
cd ~/code/jknoxdev/hack
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- toosl    ---'
cd ~/code/jknoxdev/tools
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- ctf-toos ---'
cd ~/code/jknoxdev/ctf-tools
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- ghp-site ---'
cd ~/code/jknoxdev/jknoxdev.github.io
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- ghp-site ---'
cd ~/code/jknoxdev/www.b3x9.github.io
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- ghp-site ---'
cd ~/code/jknoxdev/shop.b3x9.github.io
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- ghp-site ---'
cd ~/code/jknoxdev/blog.b3x9.github.io
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- ghp-site ---'
cd ~/code/jknoxdev/jknox.b3x9.github.io
git add .
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push
echo '--- pids-tlm ---'
cd ~/code/jknoxdev/physical-ids-telemetry.io
git commit -m "--$(echo $(hostname))--$(date '+%Y%V%w%H%M')--"
git push

echo 'waiting for all commands to complete'
wait
echo 'done!'
