
# vim: foldmethod=marker

autoload -Uz compinit && compinit

autoload -Uz colors && colors

autoload history-search-end
zle -N history-beginning-search-backward-end history-search-end
zle -N history-beginning-search-forward-end history-search-end

alias ls="ls -F --color=auto -v"
alias mv='nocorrect mv'
alias cp='nocorrect cp'
alias scp='nocorrect scp'
alias mkdir='nocorrect mkdir'

alias -g L='|less -R'
alias -g M='|more'
alias -g H='|head'
alias -g T='|tail'
alias -g G='|grep'
alias -g GI='|grep -i'

## options {{{
##  history {{{
setopt hist_no_store
setopt hist_ignore_dups
setopt hist_ignore_space
setopt inc_append_history
setopt append_history
# }}}
##  directory {{{
setopt auto_cd
setopt auto_pushd
# }}}
##  complement {{{
setopt list_packed
setopt list_types
setopt complete_aliases
setopt auto_param_keys
setopt magic_equal_subst
# }}}
##  file name and glob {{{
setopt extended_glob
setopt numeric_glob_sort
unsetopt nomatch
# }}}
## input/output {{{
setopt correct
setopt clobber
setopt correct_all
setopt ignore_eof
unsetopt flow_control
# }}}
## jobs {{{
setopt auto_resume
# }}}
## others {{{
setopt no_beep
setopt no_list_beep
setopt promptcr
setopt print_eight_bit
# }}}
# }}}
## bind key {{{
bindkey -e
bindkey "^P" history-beginning-search-backward-end
bindkey "^N" history-beginning-search-forward-end 
# }}}

# {{{ prompt
  PROMPT="%j [%{${fg[yellow]}%}%U%m%{${reset_color}%}%k%f:%~%u]% %E
%h%#>"
#}}}

export ROS_DOMAIN_ID=17
export _colcon_cd_root=/opt/ros/jazzy/

source /opt/ros/jazzy/setup.zsh
source /usr/share/colcon_cd/function/colcon_cd.sh
